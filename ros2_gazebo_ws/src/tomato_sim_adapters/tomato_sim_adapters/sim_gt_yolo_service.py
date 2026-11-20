"""Expose scene-geometry detections through the real robot's YOLO ROS service."""
from array import array
from collections import deque
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from xc_interfaces.msg import Mask
from xc_interfaces.srv import YoloDetection

from .gt_geometry_projection import (
    CLASS_NAMES, load_scene_geometry, render_depths, transform_matrix, visible_detections,
)


def _stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def depth_image_metres(message, units_per_metre):
    """Decode aligned depth with its ROS byte order and row stride."""
    if not math.isfinite(units_per_metre) or units_per_metre <= 0:
        raise ValueError('depth_units_per_metre must be finite and positive')
    scalar_types = {'16UC1': 'u2', '32FC1': 'f4'}
    if message.encoding not in scalar_types:
        raise ValueError(f'Unsupported aligned depth encoding {message.encoding}')
    dtype = np.dtype(('>' if message.is_bigendian else '<') + scalar_types[message.encoding])
    row_stride = message.step or message.width * dtype.itemsize
    if (message.width <= 0 or message.height <= 0
            or row_stride < message.width * dtype.itemsize
            or len(message.data) < row_stride * message.height):
        raise ValueError('Aligned depth image has invalid dimensions, stride, or payload')
    depth = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                       strides=(row_stride, dtype.itemsize)).astype(float)
    if message.encoding == '16UC1':
        depth /= units_per_metre
    return depth


def select_camera_info(history, image, max_age_s):
    """Select the requested optical frame's calibration at or before exposure."""
    frame = image.header.frame_id.lstrip('/')
    stamp = _stamp_ns(image.header.stamp)
    candidates = [info for info in history
                  if info.header.frame_id.lstrip('/') == frame
                  and (info.width, info.height) == (image.width, image.height)
                  and _stamp_ns(info.header.stamp) <= stamp
                  and (0 <= stamp - _stamp_ns(info.header.stamp) <= max_age_s * 10**9
                       or _stamp_ns(info.header.stamp) == 0)]
    if not frame or not candidates:
        raise ValueError(f'No exposure-matched CameraInfo for {frame} at {stamp}')
    return max(candidates, key=lambda info: _stamp_ns(info.header.stamp))


def fill_response(request, response, detections):
    """Populate complete, identically ordered YOLO arrays and full-size mono8 masks."""
    masks = []
    for detection in detections:
        pixels = np.ascontiguousarray(detection.mask, dtype=np.uint8)
        image = Image(header=request.color_image.header,
                      height=pixels.shape[0], width=pixels.shape[1], encoding='mono8',
                      is_bigendian=0, step=pixels.shape[1], data=array('B', pixels.tobytes()))
        masks.append(Mask(class_id=detection.class_id, class_names=detection.class_name,
                          confidence=detection.confidence, mask=image))
    response.color_image = request.color_image
    response.depth_image = request.depth_image
    response.class_ids = [detection.class_id for detection in detections]
    response.class_names = [detection.class_name for detection in detections]
    response.confidences = [detection.confidence for detection in detections]
    response.bboxes = [value for detection in detections for value in detection.bbox]
    response.masks = masks
    return response


class SimGroundTruthYoloService(Node):
    def __init__(self):
        super().__init__('sim_gt_yolo_service')
        scene_file = self.declare_parameter('scene_file', 'validation/phase3/phase3_scene.json').value
        model_paths = self.declare_parameter('model_paths', ['']).value
        self.units = float(self.declare_parameter('depth_units_per_metre', 1000).value)
        self.tolerance = float(self.declare_parameter('occlusion_tolerance_m', .012).value)
        self.max_info_age = float(self.declare_parameter('max_camera_info_age_s', 1.).value)
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        evidence_file = self.declare_parameter('evidence_file', '').value
        topics = self.declare_parameter('camera_info_topics', [
            '/sim/cam_left/camera_info', '/sim/cam_right/camera_info', '/sim/cam_hand/camera_info',
            '/camera/color/camera_info', '/cam_hand/color/camera_info',
        ]).value
        if not math.isfinite(self.units) or self.units <= 0:
            raise ValueError('depth_units_per_metre must be finite and positive')
        if not math.isfinite(self.tolerance) or self.tolerance < 0:
            raise ValueError('occlusion_tolerance_m must be finite and nonnegative')
        if not math.isfinite(self.max_info_age) or self.max_info_age < 0:
            raise ValueError('max_camera_info_age_s must be finite and nonnegative')
        self.geometry = load_scene_geometry(scene_file, [value for value in model_paths if value])
        self.geometry_counts = {name: sum(item.class_name == name for item in self.geometry)
                                for name in CLASS_NAMES}
        self.history = deque(maxlen=360)
        self.history_lock = threading.Lock()
        self.evidence_lock = threading.Lock()
        self.evidence_path = Path(evidence_file) if evidence_file else None
        self.evidence = {'scene_file': str(Path(scene_file).resolve()),
                         'geometry_counts': self.geometry_counts,
                         'depth_units_per_metre': self.units,
                         'occlusion_tolerance_m': self.tolerance, 'requests': []}
        self.buffer = Buffer(cache_time=Duration(seconds=30.))
        self.listener = TransformListener(self.buffer, self)
        self.callback_group = ReentrantCallbackGroup()
        self.subscriptions_info = [
            self.create_subscription(CameraInfo, topic, self.on_camera_info,
                                     qos_profile_sensor_data)
            for topic in dict.fromkeys(topics)]
        self.service = self.create_service(YoloDetection, '/yolo_detection', self.detect_objects,
                                          callback_group=self.callback_group)
        self.get_logger().info(
            f'Geometry YOLO service ready, visuals={self.geometry_counts}, '
            f'depth_units_per_metre={self.units}, tolerance_m={self.tolerance}')

    def on_camera_info(self, message):
        with self.history_lock:
            self.history.append(message)

    def detect_objects(self, request, response):
        started = time.monotonic()
        fill_response(request, response, [])
        color, depth = request.color_image, request.depth_image
        record = {'exposure_stamp_ns': _stamp_ns(color.header.stamp),
                  'optical_frame': color.header.frame_id, 'depth_encoding': depth.encoding,
                  'width': color.width, 'height': color.height}
        try:
            if (color.width, color.height) != (depth.width, depth.height):
                raise ValueError('Color and aligned depth image dimensions differ')
            if _stamp_ns(color.header.stamp) != _stamp_ns(depth.header.stamp):
                raise ValueError('Color and aligned depth exposure timestamps differ')
            if record['exposure_stamp_ns'] <= 0:
                raise ValueError('An exposure timestamp is required for geometric projection')
            if (depth.header.frame_id and depth.header.frame_id.lstrip('/')
                    != color.header.frame_id.lstrip('/')):
                raise ValueError('Aligned depth and color optical frames differ')
            with self.history_lock:
                info = select_camera_info(list(self.history), color, self.max_info_age)
            if any(abs(value) > 1e-12 for value in info.d):
                raise ValueError('Geometry projector requires the Gazebo pinhole calibration')
            camera = np.asarray(info.k).reshape((3, 3))
            transform = self.buffer.lookup_transform(
                color.header.frame_id.lstrip('/'), self.world_frame,
                Time.from_msg(color.header.stamp, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=.25)).transform
            optical_from_world = transform_matrix(
                [transform.translation.x, transform.translation.y, transform.translation.z],
                [transform.rotation.x, transform.rotation.y,
                 transform.rotation.z, transform.rotation.w])
            observed = depth_image_metres(depth, self.units)
            projected = render_depths(self.geometry, optical_from_world, camera,
                                      color.width, color.height)
            detections = visible_detections(projected, observed, self.tolerance)
            fill_response(request, response, detections)
            by_name = {detection.class_name: detection for detection in detections}
            record.update({
                'camera_info_stamp_ns': _stamp_ns(info.header.stamp),
                'camera_matrix': camera.tolist(),
                'optical_from_world': optical_from_world.tolist(),
                'valid_observed_depth_pixels': int(np.count_nonzero(
                    np.isfinite(observed) & (observed > 0))),
                'classes': {name: {
                    'projected_pixels': int(np.count_nonzero(np.isfinite(projected[name]))),
                    'visible_pixels': int(np.count_nonzero(by_name[name].mask)) if name in by_name else 0,
                    'bbox_xyxy': by_name[name].bbox if name in by_name else None,
                    'max_depth_residual_m': by_name[name].max_depth_residual_m if name in by_name else None,
                } for name in CLASS_NAMES},
            })
            self.get_logger().info(
                f'Geometry detections frame={color.header.frame_id}, '
                f'stamp_ns={record["exposure_stamp_ns"]}, '
                f'visible_pixels={[(name, values["visible_pixels"]) for name, values in record["classes"].items()]}')
        except Exception as error:
            record['error'] = str(error)
            self.get_logger().error(f'Geometry detection failed: {error}')
        record['elapsed_s'] = time.monotonic() - started
        self.write_evidence(record)
        return response

    def write_evidence(self, record):
        if self.evidence_path is None:
            return
        try:
            with self.evidence_lock:
                self.evidence['requests'].append(record)
                self.evidence['requests'] = self.evidence['requests'][-100:]
                self.evidence_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.evidence_path.with_name(self.evidence_path.name + '.tmp')
                temporary.write_text(json.dumps(self.evidence, indent=2, allow_nan=False) + '\n')
                temporary.replace(self.evidence_path)
        except OSError as error:
            self.get_logger().error(f'Could not write geometry detection evidence: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = SimGroundTruthYoloService()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
