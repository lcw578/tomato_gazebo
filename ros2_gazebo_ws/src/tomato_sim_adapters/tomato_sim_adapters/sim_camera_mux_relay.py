"""Republish synchronized Gazebo RGB-D on the real camera topics."""
from array import array

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

STREAMS = {'image_raw': Image, 'depth/image_raw': Image, 'camera_info': CameraInfo}


def depth_to_uint16(message, units_per_metre):
    if message.encoding != '32FC1':
        raise ValueError(f'Expected 32FC1, got {message.encoding}')
    depth = np.ndarray(
        (message.height, message.width),
        dtype='>f4' if message.is_bigendian else '<f4',
        buffer=message.data, strides=(message.step, 4))
    valid = np.isfinite(depth) & (depth > 0) & (depth <= 65535 / units_per_metre)
    converted = np.zeros(depth.shape, dtype='<u2')
    converted[valid] = np.maximum(1, np.rint(depth[valid] * units_per_metre)).astype('<u2')
    output = Image(header=message.header, height=message.height, width=message.width,
                   encoding='16UC1', is_bigendian=0, step=2 * message.width)
    output.data = array('B', converted.tobytes())
    return output


class CameraMux(Node):
    def __init__(self):
        super().__init__('sim_camera_mux_relay')
        self.side = self.declare_parameter('active_side', 'left').value
        self.remote_units_per_metre = self.declare_parameter('remote_depth_units_per_metre', 4000).value
        if self.remote_units_per_metre <= 0:
            raise ValueError('remote_depth_units_per_metre must be positive')
        if self.side not in ('left', 'right'):
            raise ValueError('active_side must be left or right')
        self.cutover = -1
        self.pending = {'left': {}, 'right': {}, 'hand': {}}
        self.last_stamp = {'left': -1, 'right': -1, 'hand': -1}
        self.remote = self.make_publishers('/camera', 'image_raw')
        self.hand = self.make_publishers('/cam_hand', 'image_rect_raw')
        self.hand_rgb = self.create_publisher(Image, '/cam_hand/color/image_raw', 10)
        for name in self.pending:
            for topic, message_type in STREAMS.items():
                self.create_subscription(
                    message_type, f'/sim/cam_{name}/{topic}',
                    lambda msg, camera=name, stream=topic: self.relay(camera, stream, msg),
                    qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.set_side)

    def make_publishers(self, root, image):
        return {
            'image_raw': self.create_publisher(Image, f'{root}/color/{image}', 10),
            'depth/image_raw': self.create_publisher(
                Image, f'{root}/aligned_depth_to_color/image_raw', 10),
            'camera_info': self.create_publisher(CameraInfo, f'{root}/color/camera_info', 10),
        }

    def set_side(self, parameters):
        for parameter in parameters:
            if parameter.name == 'active_side':
                if parameter.value not in ('left', 'right'):
                    return SetParametersResult(successful=False, reason='active_side must be left or right')
                if parameter.value != self.side:
                    self.side = parameter.value
                    self.cutover = self.get_clock().now().nanoseconds
                    self.pending['left'].clear()
                    self.pending['right'].clear()
        return SetParametersResult(successful=True)

    def relay(self, camera, stream, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if camera != 'hand' and (camera != self.side or stamp <= self.cutover):
            return
        if stamp <= self.last_stamp[camera]:
            return
        queue = self.pending[camera]
        group = queue.setdefault(stamp, {})
        group[stream] = msg
        while len(queue) > 4:
            del queue[min(queue)]
        if len(group) != len(STREAMS) or stamp not in queue:
            return
        hand = camera == 'hand'
        try:
            group['depth/image_raw'] = depth_to_uint16(
                group['depth/image_raw'], 1000 if hand else self.remote_units_per_metre)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)
            del queue[stamp]
            return
        output = self.hand if hand else self.remote
        for topic, message in group.items():
            output[topic].publish(message)
        if hand:
            self.hand_rgb.publish(group['image_raw'])
        self.last_stamp[camera] = stamp
        for old in tuple(queue):
            if old <= stamp:
                del queue[old]


def main():
    rclpy.init()
    node = CameraMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
