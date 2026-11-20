#!/usr/bin/env python3
"""Measure the robot on the actual v4 rails without commanding a harvest."""
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PILImage
from PIL.PngImagePlugin import PngInfo
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from sensor_msgs.msg import Image, JointState
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

from tomato_sim_adapters.arm_kinematics import JOINTS, OEM_HOME

from simulation_config import load_configuration, read_world, sha256_file, workspace

ROOT = workspace()


class BaselinePreflight(Node):
    def __init__(self, output):
        super().__init__('verify_v4_baseline', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.baseline = load_configuration()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.joints = None
        self.samples = []
        self.rgb = None
        self.depth = None
        self.overview = None
        self.summary = {'baseline': {key: value for key, value in self.baseline.items()
                                     if key not in ('robot_root', 'asset_root', 'yolo_weights',
                                                    'ros_python', 'yolo_python')}, 'passed': False,
                        'scope': 'static assembly and live Gazebo camera only; no harvest acceptance'}
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 lambda msg: setattr(self, 'rgb', msg), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                 lambda msg: setattr(self, 'depth', msg), qos_profile_sensor_data)
        self.create_subscription(Image, '/sim/validation/overview/overview/image_raw',
                                 lambda msg: setattr(self, 'overview', msg), qos_profile_sensor_data)
        self.state = self.create_client(GetEntityState, '/get_entity_state')
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.parameters = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')

    def on_joints(self, message):
        if not set(JOINTS + ['machine_slide']) <= set(message.name):
            return
        self.joints = message
        self.samples.append((time.monotonic(), message))
        self.samples = [(t, value) for t, value in self.samples
                        if time.monotonic() - t < 2.0]

    def wait(self, predicate, description, timeout=90):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            value = predicate()
            if value is not None and value is not False:
                return value
        raise RuntimeError(f'Timeout: {description}')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('Gazebo service unavailable')
        future = client.call_async(request)
        self.wait(future.done, 'Gazebo response', 30)
        response = future.result()
        if not response.success:
            raise RuntimeError(getattr(response, 'status_message', 'Gazebo request failed'))
        return response

    def transform(self, target, source):
        def lookup():
            try:
                return self.tf.lookup_transform(target, source, rclpy.time.Time()).transform
            except TransformException:
                return None
        return self.wait(lookup, f'TF {target} <- {source}')

    @staticmethod
    def rpy(transform):
        q = transform.rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')

    def save_image(self, message, name, topic):
        if message.encoding not in ('rgb8', 'bgr8'):
            raise RuntimeError(f'Unexpected RGB encoding {message.encoding}')
        data = np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                          buffer=message.data, strides=(message.step, 3, 1)).copy()
        if message.encoding == 'bgr8':
            data = data[:, :, ::-1]
        metadata = PngInfo()
        metadata.add_text('source', 'Gazebo Classic 11; live ROS Image')
        metadata.add_text('world', self.baseline['world'])
        metadata.add_text('topic', topic)
        metadata.add_text('world_sha256', self.summary['source_world_sha256'])
        path = self.output / name
        PILImage.fromarray(data).save(path, pnginfo=metadata)
        return {'file': str(path), 'topic': topic, 'frame_id': message.header.frame_id,
                'stamp': {'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec},
                'dimensions': [message.width, message.height], 'engine': 'Gazebo Classic 11'}

    def add_overview_camera(self):
        eye, target = np.array([3.0, -3.0, 2.8]), np.array([0.0, 0.0, 1.0])
        direction = target - eye
        yaw = math.atan2(direction[1], direction[0])
        pitch = -math.atan2(direction[2], math.hypot(*direction[:2]))
        xml = f'''<sdf version="1.7"><model name="v4_validation_camera">
          <static>true</static><pose>{eye[0]} {eye[1]} {eye[2]} 0 {pitch} {yaw}</pose>
          <link name="camera_link"><sensor name="overview" type="camera">
          <always_on>true</always_on><update_rate>2</update_rate>
          <camera><horizontal_fov>1.1</horizontal_fov>
          <image><width>960</width><height>600</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>30</far></clip></camera>
          <plugin name="overview_ros" filename="libgazebo_ros_camera.so">
          <ros><namespace>/sim/validation/overview</namespace></ros>
          <camera_name>overview</camera_name><frame_name>v4_validation_camera</frame_name>
          </plugin></sensor></link></model></sdf>'''
        self.call(self.spawn, SpawnEntity.Request(name='v4_validation_camera', xml=xml,
                                                  reference_frame='world'))

    def run(self):
        record = self.output / 'world.json'
        self.wait(record.is_file, 'launched SDF world record')
        launch = json.loads(record.read_text())
        dependencies = json.loads((self.output / 'dependencies.json').read_text())
        source = Path(launch['source_world'])
        source_hash = sha256_file(source)
        if (str(source) != dependencies['map']['world']
                or source_hash != launch['source_sha256']
                or source_hash != dependencies['map']['sha256']):
            raise RuntimeError('The launched map differs from the validated dependency record')
        _, world = read_world(source)
        self.summary['dependencies'] = dependencies
        self.summary['launch_world'] = launch
        self.summary['source_world_sha256'] = source_hash
        self.summary['world_name'] = launch['world_name']
        self.summary['plant_count'] = sum((x.findtext('name') or '').startswith('plant_')
                                          for x in world.findall('include'))
        self.wait(lambda: self.joints is not None and self.rgb is not None
                  and self.depth is not None, 'robot and live Gazebo RGB-D')
        if not self.parameters.wait_for_service(timeout_sec=10):
            raise RuntimeError('robot_state_publisher parameter service unavailable')
        future = self.parameters.call_async(GetParameters.Request(names=['robot_description']))
        self.wait(future.done, 'assembled robot_description')
        urdf = ET.fromstring(future.result().values[0].string_value)
        mount = urdf.find("joint[@name='arm_mount']/origin")
        mount_rpy = np.array(list(map(float, mount.get('rpy').split())))
        self.summary['arm_mount_rpy_deg'] = np.rad2deg(mount_rpy).tolist()
        self.wait(lambda: len(self.samples) > 2
                  and self.samples[-1][0] - self.samples[0][0] >= 1.5, 'stationary sample window')
        errors, speeds, rails = [], [], []
        for _, msg in self.samples:
            indices = [msg.name.index(name) for name in JOINTS]
            errors.append(max(abs(msg.position[i] - q) for i, q in zip(indices, OEM_HOME)))
            speeds.append(max(abs(msg.velocity[i]) for i in indices))
            rails.append(msg.position[msg.name.index('machine_slide')])
        carriage = self.transform('world', 'robot_carriage')
        carriage_rpy = self.rpy(carriage)
        base = self.transform('world', 'arm_base_link')
        q = base.rotation
        self.summary['carriage'] = {
            'world_xyz_m': [carriage.translation.x, carriage.translation.y, carriage.translation.z],
            'tf_world_rpy_deg': np.rad2deg(carriage_rpy).tolist(),
            'cad_datum_note': 'chassis_cad_datum has an intentional pi roll; carriage is the level reference'}
        self.summary['arm_base_world_quaternion_xyzw'] = [q.x, q.y, q.z, q.w]
        self.summary['stationary_window'] = {
            'wall_duration_s': self.samples[-1][0] - self.samples[0][0],
            'samples': len(self.samples), 'max_home_error_rad': max(errors),
            'max_arm_velocity_rad_s': max(speeds), 'rail_min_max_m': [min(rails), max(rails)]}
        track = self.call(self.state, GetEntityState.Request(name=self.baseline['track_entity'],
                                                            reference_frame='world')).state
        self.summary['track_entity_present'] = True
        self.summary['track_entity_world_xyz_m'] = [track.pose.position.x,
                                                   track.pose.position.y, track.pose.position.z]
        physical = self.call(self.state, GetEntityState.Request(name='tomato_picker::robot_carriage',
                                                                reference_frame='world')).state
        qp = physical.pose.orientation
        physical_rpy = Rotation.from_quat([qp.x, qp.y, qp.z, qp.w]).as_euler('xyz')
        self.summary['carriage']['gazebo_world_rpy_deg'] = np.rad2deg(physical_rpy).tolist()
        self.summary['carriage']['gazebo_world_xyz_m'] = [physical.pose.position.x,
                                                         physical.pose.position.y,
                                                         physical.pose.position.z]
        self.summary['scene_camera_rgb'] = self.save_image(self.rgb, 'gazebo_v4_rgb.png',
                                                          '/camera/color/image_raw')
        self.summary['depth_encoding'] = self.depth.encoding
        self.add_overview_camera()
        self.wait(lambda: self.overview is not None, 'live Gazebo overview camera', 30)
        self.summary['overview'] = self.save_image(self.overview, 'gazebo_v4_robot_overview.png',
                                                   '/sim/validation/overview/overview/image_raw')
        self.summary['passed'] = bool(
            self.summary['world_name'] == self.baseline['baseline_id']
            and self.summary['plant_count'] == self.baseline['plant_count']
            and abs(math.remainder(mount_rpy[2] - math.radians(self.baseline['arm_base_yaw_deg']), math.tau)) < 1e-6
            and np.max(np.abs(carriage_rpy[:2])) < math.radians(1)
            and np.max(np.abs(physical_rpy[:2])) < math.radians(1)
            and abs(physical.pose.position.y) < 0.005
            and abs(physical.pose.position.z - self.baseline['rail_top_z_m']) < 0.005
            and abs(carriage.translation.y) < 0.005
            and abs(carriage.translation.z - self.baseline['rail_top_z_m']) < 0.005
            and max(errors) < 0.08 and max(speeds) < 0.12
            and max(rails) - min(rails) < 0.005
            and self.depth.encoding == '16UC1')
        return self.summary['passed']


def main():
    rclpy.init()
    node = BaselinePreflight(sys.argv[1] if len(sys.argv) > 1 else ROOT / 'validation/v4_baseline')
    try:
        passed = node.run()
    except Exception as error:
        node.summary['error'] = str(error)
        passed = False
    finally:
        (node.output / 'summary.json').write_text(json.dumps(node.summary, indent=2) + '\n')
        (node.output / 'status.txt').write_text('PASS\n' if node.summary['passed'] else 'FAIL\n')
        print(json.dumps(node.summary, indent=2))
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
