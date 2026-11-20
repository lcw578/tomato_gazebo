#!/usr/bin/env python3
"""Measure the live vision-to-harvest chain driven by the project's ROS controller."""
import json
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import UInt8
from tf2_ros import Buffer, TransformException, TransformListener
from scipy.spatial.transform import Rotation
from xc_interfaces.msg import Detection2UI, TestResult


from ament_index_python.packages import get_package_share_directory

from simulation_config import workspace

ROOT = workspace()
SCENE = json.loads((ROOT / 'validation/phase3/phase3_scene.json').read_text())
RUNTIME = ROOT / 'validation/phase4_runtime.json'


def read_runtime():
    try:
        return json.loads(RUNTIME.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def transform_point(transform, point):
    q = transform.rotation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    translation = np.array([transform.translation.x, transform.translation.y,
                            transform.translation.z])
    return translation + rotation.apply(point)


class Phase4Checks(Node):
    def __init__(self, output):
        super().__init__('verify_phase4', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.camera_info = None
        self.rgb_frames = 0
        self.rgb_frame_file = None
        self.detection_messages = 0
        self.peak_class_counts = {name: 0 for name in ('stem', 'tomato', 'trunk')}
        self.detections = []
        self.feedback = []
        self.x1 = []
        self.joints = None
        self.started = time.monotonic()
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 lambda msg: setattr(self, 'camera_info', msg), 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_rgb, 10)
        self.create_subscription(Detection2UI, '/detection2ui', self.on_detection, 10)
        self.create_subscription(TestResult, '/test_result',
                                 lambda msg: self.feedback.append((time.monotonic(), int(msg.result))), 10)
        self.create_subscription(UInt8, '/gripper/input_x1_status',
                                 lambda msg: self.x1.append((time.monotonic(), int(msg.data))), 10)
        self.create_subscription(JointState, '/joint_states',
                                 lambda msg: setattr(self, 'joints', msg), qos_profile_sensor_data)

    def on_detection(self, message):
        self.detection_messages += 1
        for name, peak in self.peak_class_counts.items():
            self.peak_class_counts[name] = max(peak, list(message.class_names).count(name))
        if message.picking_points_z <= 0 or not np.isfinite(message.picking_points_z):
            return
        self.detections.append((time.monotonic(), message))

    def on_rgb(self, message):
        self.rgb_frames += 1
        if self.rgb_frame_file is not None or message.encoding != 'rgb8':
            return
        image = np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                           buffer=message.data, strides=(message.step, 3, 1))
        image_path = self.output.parent / 'phase4_rgb.png'
        PILImage.fromarray(image).save(image_path)
        self.rgb_frame_file = image_path

    def wait_for(self, predicate, label, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f'timed out waiting for {label}')

    def lookup(self, target, source):
        end = time.monotonic() + 30
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                return self.tf.lookup_transform(target, source, rclpy.time.Time()).transform
            except TransformException:
                pass
        raise RuntimeError(f'Missing TF {target} <- {source}')

    def measure_detection(self, message):
        if self.camera_info is None:
            raise RuntimeError('CameraInfo was not received')
        matrix_path = ROOT / 'validation/phase4/calibration/right/calibration_results.txt'
        matrix = np.loadtxt(matrix_path, delimiter=',')
        u, v, depth = (float(message.picking_points_x), float(message.picking_points_y),
                       float(message.picking_points_z))
        if not (0 <= u < self.camera_info.width and 0 <= v < self.camera_info.height):
            raise RuntimeError(f'Detection pixel outside image: {(u, v)}')
        k = self.camera_info.k
        optical = np.array([(u - k[2]) * depth / k[0],
                            (v - k[5]) * depth / k[4], depth, 1.0])
        base_point = (matrix @ optical)[:3]
        truth_world = np.array(SCENE['cut_point_world'], dtype=float)
        base_from_world = self.lookup('arm_base_link', 'world')
        truth_base = transform_point(base_from_world, truth_world)
        return {
            'pixel_uv': [u, v], 'depth_m': depth,
            'vision_cut_point_arm_base_m': base_point.tolist(),
            'ground_truth_cut_point_arm_base_m': truth_base.tolist(),
            'vision_error_mm': float(np.linalg.norm(base_point - truth_base) * 1000.0),
        }

    def run(self, timeout):
        self.wait_for(lambda: self.camera_info is not None and self.joints is not None,
                      'RGB-D camera and joint states', 90)
        deadline = time.monotonic() + timeout
        held_time = None
        released_time = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            runtime = read_runtime()
            if runtime.get('state') == 'HELD_BY_GRIPPER' and held_time is None:
                held_time = time.monotonic()
            if runtime.get('state') in ('RELEASED', 'IN_BASKET'):
                released_time = time.monotonic()
                break
        runtime = read_runtime()
        detection = next((record for record in self.detections if record[1].picking_points_z > 0), None)
        detection_metrics = self.measure_detection(detection[1]) if detection else None
        attempts = runtime.get('cut_attempts', [])
        cut = next((item for item in attempts if item.get('event') == 'CUT_SUCCESS'), None)
        release = runtime.get('release', {})
        positive_feedback = next((stamp for stamp, result in self.feedback if result == 2), None)
        stem_feedback = next((stamp for stamp, result in self.feedback if result == 3), None)
        x1_stamp = next((stamp for stamp, value in self.x1 if value == 1), None)
        main_contacts = int(runtime.get('main_stem_contacts', 0))
        summary = {
            'phase': 4,
            'baseline_id': SCENE['baseline_id'],
            'source_world': SCENE['source_world'],
            'source_world_sha256': SCENE['source_world_sha256'],
            'model_path': str(Path(get_package_share_directory('xc_robot_main_control')) / 'weights/best.pt'),
            'model_classes': ['stem', 'tomato', 'trunk'],
            'moge_depth_bypassed': True,
            'depth_source': '/camera/aligned_depth_to_color/image_raw',
            'tcp_reference': 'tool_tcp; adapter reverses the main controller 0.10 m request offset',
            'vision': detection_metrics,
            'rgb_frame_count': self.rgb_frames,
            'rgb_frame_file': str(self.rgb_frame_file) if self.rgb_frame_file else None,
            'detection2ui_messages': self.detection_messages,
            'peak_detection_boxes_per_class': self.peak_class_counts,
            'test_result_2_received': positive_feedback is not None,
            'test_result_3_received': stem_feedback is not None,
            'x1_triggered': x1_stamp is not None,
            'cut_success': cut is not None,
            'tcp_final_error_mm': (float(cut.get('tcp_cutpoint_error_m', float('inf'))) * 1000.0
                                   if cut else None),
            'main_stem_contacts': main_contacts,
            'runtime_state': runtime.get('state'),
            'release': release,
            'stage_times_s': {
                'detection_to_feedback': (positive_feedback - detection[0]
                                          if detection and positive_feedback else None),
                'feedback_to_cut': (held_time - positive_feedback
                                    if held_time and positive_feedback else None),
                'cut_to_release': (released_time - held_time
                                   if released_time and held_time else None),
                'detection_to_release': (released_time - detection[0]
                                         if detection and released_time else None),
            },
            'passed': bool(
                detection_metrics is not None and positive_feedback is not None
                and x1_stamp is not None and cut is not None and released_time is not None
                and main_contacts == 0 and cut.get('tcp_cutpoint_error_m', 1.0) <= 0.005),
        }
        self.output.write_text(json.dumps(summary, indent=2) + '\n')
        return summary['passed']


def main():
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'validation/phase4_summary.json'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 480.0
    rclpy.init()
    node = Phase4Checks(output)
    try:
        passed = node.run(timeout)
    except Exception as error:
        summary = read_runtime()
        summary.update({'phase': 4, 'passed': False, 'error': str(error),
                        'model_classes': ['stem', 'tomato', 'trunk'],
                        'moge_depth_bypassed': True})
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, indent=2) + '\n')
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
