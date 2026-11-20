#!/usr/bin/env python3
"""Exercise the real ROS service path and capture Phase 3 evidence."""
import json
import math
from collections import deque
from pathlib import Path
import sys
import time

import numpy as np
import PyKDL as kdl
import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import UInt8
from tf2_ros import Buffer, TransformException, TransformListener
from xc_interfaces.msg import MidpointReached, TestResult
from xc_interfaces.srv import ControlAGV, GripperControl, MoveJoints, MoveToPose

from tomato_sim_adapters.arm_kinematics import OEM_HOME

from simulation_config import harvest_launch_defaults, load_configuration, workspace

RAIL_TRAVEL_SPEED_MM_S = 100

WORKSPACE = workspace()
SCENE = json.loads((WORKSPACE / 'validation/phase3/phase3_scene.json').read_text())
TARGET = np.array(SCENE['cut_point_world'], dtype=float)
TANGENT = np.array(SCENE['peduncle_tangent_world'], dtype=float)
STEM_POINT = np.array(SCENE['main_stem_probe_world'])
STEM_TANGENT = np.array(SCENE['main_stem_tangent_world'])


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def rotate(q, vector):
    return Rotation.from_quat(q).apply(vector)


def pose_from_entity(state):
    pose = state.pose
    return (np.array([pose.position.x, pose.position.y, pose.position.z]),
            Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w]))


def image_array(message):
    if message.encoding in ('rgb8', 'bgr8'):
        return np.ndarray((message.height, message.width, 3), dtype=np.uint8,
                          buffer=message.data, strides=(message.step, 3, 1)).copy()
    if message.encoding == '16UC1':
        return np.ndarray((message.height, message.width), dtype='<u2',
                          buffer=message.data, strides=(message.step, 2)).copy()
    if message.encoding == '32FC1':
        return np.ndarray((message.height, message.width), dtype='<f4',
                          buffer=message.data, strides=(message.step, 4)).copy()
    raise ValueError(f'Unsupported camera encoding: {message.encoding}')


class Phase3Checks(Node):
    def __init__(self, output):
        super().__init__('verify_phase3', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.summary = {'phase': 3, 'target': SCENE['target_model'], 'checks': {}, 'stage_times_s': {},
                        'baseline_id': SCENE['baseline_id'], 'source_world': SCENE['source_world'],
                        'source_world_sha256': SCENE['source_world_sha256']}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.joints = {}
        self.joint_velocities = {}
        self.slide_history = deque()
        self.rgb = None
        self.depth = None
        self.info = None
        self.x1_values = []
        self.feedback = []
        self.midpoints = []
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(UInt8, '/gripper/input_x1_status',
                                 lambda msg: self.x1_values.append((time.monotonic(), msg.data)), 10)
        self.create_subscription(TestResult, '/test_result',
                                 lambda msg: self.feedback.append((time.monotonic(), msg.result)), 10)
        self.create_subscription(MidpointReached, '/midpoint_reached',
                                 lambda msg: self.midpoints.append(msg.reached), 10)
        self.move_pose = self.create_client(MoveToPose, '/move_to_pose')
        self.move_joints = self.create_client(MoveJoints, '/move_joints')
        self.retreat = self.create_client(MoveToPose, '/target_return_midpoint')
        self.return_box = self.create_client(MoveToPose, '/return_to_box')
        self.gripper = self.create_client(GripperControl, '/gripper/control')
        self.control_agv = self.create_client(ControlAGV, '/control_agv')
        self.get_entity = self.create_client(GetEntityState, '/get_entity_state')

    def on_joints(self, message):
        self.joints = dict(zip(message.name, message.position))
        self.joint_velocities = dict(zip(message.name, message.velocity))
        if 'machine_slide' in self.joints:
            now = time.monotonic()
            self.slide_history.append((now, self.joints['machine_slide']))
            while self.slide_history and now - self.slide_history[0][0] > 2.0:
                self.slide_history.popleft()

    def on_rgb(self, message):
        self.rgb = message

    def on_depth(self, message):
        self.depth = message

    def on_info(self, message):
        self.info = message

    def wait_for(self, predicate, label, timeout=30):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.04)
            value = predicate()
            if value is not None and value is not False:
                return value
        raise RuntimeError(f'timed out waiting for {label}')

    def call(self, client, request, label, timeout=60):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'{label} service unavailable')
        future = client.call_async(request)
        self.wait_for(future.done, label, timeout)
        response = future.result()
        if hasattr(response, 'result') and response.result != 0:
            raise RuntimeError(f'{label} failed with result={response.result}')
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(f'{label} failed: {getattr(response, "message", "")}' )
        return response

    def transform(self, target, source):
        def lookup():
            try:
                return self.tf.lookup_transform(target, source, rclpy.time.Time()).transform
            except TransformException:
                return None
        return self.wait_for(lookup, f'TF {target} <- {source}')

    def entity_state(self, name):
        return self.call(self.get_entity, GetEntityState.Request(name=name, reference_frame='world'),
                         f'GetEntityState {name}', timeout=10).state

    def set_slide(self, target):
        self.wait_for(lambda: 'machine_slide' in self.joints, 'rail joint state')
        self.call(self.control_agv,
                  ControlAGV.Request(linear_speed=0, angular_speed=0,
                                     lift_magnetic_sensor=False),
                  'stop rail before station move', timeout=10)
        def rail_stable():
            if len(self.slide_history) < 2:
                return False
            elapsed = self.slide_history[-1][0] - self.slide_history[0][0]
            delta = self.slide_history[-1][1] - self.slide_history[0][1]
            return elapsed >= 0.75 and abs(delta) <= 0.005
        self.wait_for(rail_stable, 'rail position settles', timeout=60)
        start = self.joints['machine_slide']
        offset = abs(start - target)
        self.summary['rail_start_position_m'] = start
        self.summary['rail_requested_position_m'] = target
        self.summary['rail_station_error_m'] = offset
        if offset <= 0.05:
            self.summary['rail_position_m'] = start
            self.summary['rail_station_accepted_within_tolerance'] = True
            return
        direction = 1.0 if target > start else -1.0
        request = ControlAGV.Request(linear_speed=int(direction * RAIL_TRAVEL_SPEED_MM_S), angular_speed=0,
                                     lift_magnetic_sensor=False)
        try:
            response = self.call(self.control_agv, request, 'move rail through /control_agv', timeout=10)
            if not response.control_cmd_success:
                raise RuntimeError('ControlAGV rejected the rail movement command')
            self.wait_for(lambda: (self.joints.get('machine_slide', start) - target) * direction >= 0,
                          f'rail reaches {target:.3f} m', timeout=360)
        finally:
            self.call(self.control_agv,
                      ControlAGV.Request(linear_speed=0, angular_speed=0,
                                         lift_magnetic_sensor=False),
                      'stop rail through /control_agv', timeout=10)
        self.wait_for(lambda: rail_stable(), 'rail position settles after move', timeout=60)
        self.wait_for(lambda: abs(self.joints.get('machine_slide', start) - target) < 0.02,
                      'rail within 20 mm of station', timeout=10)
        self.summary['rail_position_m'] = self.joints['machine_slide']
        self.summary['rail_station_accepted_within_tolerance'] = False

    def set_home(self):
        request = MoveJoints.Request()
        request.joints = list(map(float, OEM_HOME))
        self.call(self.move_joints, request, 'move to OEM home', timeout=240)

    def assert_home_start(self):
        names = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                 'wrist1_joint', 'wrist2_joint', 'wrist3_joint')
        self.wait_for(lambda: all(name in self.joints for name in names), 'arm joint states')
        current = np.array([self.joints[name] for name in names])
        error = float(np.max(np.abs(current - np.asarray(OEM_HOME))))
        self.summary['initial_arm_max_error_rad'] = error
        self.summary['initial_arm_joint_names'] = list(names)
        self.summary['initial_arm_joint_positions_rad'] = current.tolist()
        if error > 0.08:
            raise RuntimeError(f'Initial arm pose is not near OEM home: {error:.4f} rad')

    def world_tcp_transform(self, after_stamp=None):
        # A service reply may precede queued TF messages. Never grade the
        # previous midpoint pose as the final TCP measurement.
        def lookup():
            try:
                message = self.tf.lookup_transform('world', 'tool_tcp', rclpy.time.Time())
                stamp = rclpy.time.Time.from_msg(message.header.stamp)
                if after_stamp is not None and stamp.nanoseconds < after_stamp.nanoseconds:
                    return None
                return message.transform
            except TransformException:
                return None
        value = self.wait_for(lookup, 'fresh world <- tool_tcp transform')
        t, q = value.translation, value.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])

    def target_pose(self, point_world, tangent_world):
        arm = self.transform('world', 'arm_base_link')
        base_position = np.array([arm.translation.x, arm.translation.y, arm.translation.z])
        base_rotation = Rotation.from_quat([arm.rotation.x, arm.rotation.y,
                                            arm.rotation.z, arm.rotation.w])
        x_axis = unit(tangent_world)
        z_axis = base_position - point_world
        z_axis -= x_axis * float(np.dot(z_axis, x_axis))
        z_axis = unit(z_axis)
        y_axis = unit(np.cross(z_axis, x_axis))
        z_axis = unit(np.cross(x_axis, y_axis))
        world_rotation = Rotation.from_matrix(np.column_stack((x_axis, y_axis, z_axis)))
        local_position = base_rotation.inv().apply(point_world - base_position)
        local_rotation = base_rotation.inv() * world_rotation
        return local_position, local_rotation

    def move_to(self, point_world, tangent_world, *, expect_feedback=None, timeout=300):
        local_position, local_rotation = self.target_pose(point_world, tangent_world)
        rx, ry, rz = local_rotation.as_euler('xyz')
        request = MoveToPose.Request(x=float(local_position[0]), y=float(local_position[1]),
                                     z=float(local_position[2]), rx=float(rx), ry=float(ry), rz=float(rz),
                                     mid_to_target=False, mid2target_angle=0.0, start2mid=True)
        self.call(self.move_pose, request, 'move to harvest midpoint', timeout)
        request.mid_to_target = True
        request.start2mid = False
        self.call(self.move_pose, request, 'approach harvest target', timeout)
        tcp_position, _ = self.world_tcp_transform(after_stamp=self.get_clock().now())
        position_error = float(np.linalg.norm(tcp_position - point_world))
        if position_error > 0.005:
            raise RuntimeError(f'Actual tool_tcp missed requested pose by {position_error:.6f} m')
        if expect_feedback is not None:
            self.wait_for(lambda: any(value == expect_feedback for _, value in self.feedback),
                          f'test_result {expect_feedback}', timeout=5)
        return request, position_error

    def command_gripper(self, command):
        return self.call(self.gripper, GripperControl.Request(command=command),
                         f'gripper {command}', timeout=10)

    def return_to_box_pose(self):
        self.call(self.return_box, MoveToPose.Request(), 'return to basket', timeout=300)

    def camera_snapshot(self):
        self.wait_for(lambda: self.rgb is not None and self.depth is not None and self.info is not None,
                      'RGB-D camera frames')
        return self.rgb, self.depth, self.info

    def camera_removal_check(self, before, after, point_world):
        rgb0, depth0, info0 = before
        rgb1, depth1, info1 = after
        frame = rgb0.header.frame_id
        tf = self.transform('world', frame)
        cam_position = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        cam_rotation = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
        point_cam = cam_rotation.inv().apply(point_world - cam_position)
        if point_cam[2] <= 0:
            return {'visible': False, 'reason': 'target behind camera'}
        u = int(round(info0.k[0] * point_cam[0] / point_cam[2] + info0.k[2]))
        v = int(round(info0.k[4] * point_cam[1] / point_cam[2] + info0.k[5]))
        if not (0 <= u < rgb0.width and 0 <= v < rgb0.height):
            return {'visible': False, 'reason': 'target outside camera image', 'pixel': [u, v]}
        a, b = image_array(rgb0), image_array(rgb1)
        da, db = image_array(depth0).astype(np.float32), image_array(depth1).astype(np.float32)
        radius = 20
        ys = slice(max(0, v-radius), min(rgb0.height, v+radius+1))
        xs = slice(max(0, u-radius), min(rgb0.width, u+radius+1))
        color_delta = np.abs(a[ys, xs].astype(np.int16) - b[ys, xs].astype(np.int16))
        depth_delta = np.abs(da[ys, xs] - db[ys, xs])
        depth_scale = 4000.0 if depth0.encoding == '16UC1' else 1.0
        expected = point_cam[2] * depth_scale
        occupied_before = float(np.mean((da[ys, xs] > 0) & (np.abs(da[ys, xs] - expected) < 0.35 * depth_scale)))
        occupied_after = float(np.mean((db[ys, xs] > 0) & (np.abs(db[ys, xs] - expected) < 0.35 * depth_scale)))
        return {
            'visible': True, 'pixel': [u, v], 'expected_depth_m': float(point_cam[2]),
            'rgb_changed_fraction': float(np.mean(np.any(color_delta > 20, axis=2))),
            'depth_changed_fraction': float(np.mean(depth_delta > 0.12 * depth_scale)),
            'depth_occupancy_before': occupied_before, 'depth_occupancy_after': occupied_after,
            'rgb_dimensions': [rgb0.width, rgb0.height],
            'depth_encoding': depth0.encoding,
        }

    def run(self, rail_station):
        self.wait_for(lambda: self.rgb is not None and 'machine_slide' in self.joints,
                      'simulator and camera ready', timeout=90)
        self.set_slide(rail_station)
        self.assert_home_start()
        before_state = self.entity_state(SCENE['target_model'])
        before = self.camera_snapshot()
        fruit_before = pose_from_entity(before_state)
        # Miss case: 35 mm away from the cut location must not enable X1 or detach fruit.
        miss_request, miss_error = self.move_to(TARGET + TANGENT * 0.035, TANGENT)
        miss_tcp, _ = self.world_tcp_transform()
        miss_cutpoint_error = float(np.linalg.norm(miss_tcp - TARGET))
        x1_at_miss = any(value == 1 for _, value in self.x1_values[-20:])
        self.command_gripper('close')
        self.wait_for(lambda: any(item.get('event') == 'CUT_MISS'
                                  for item in json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text()).get('cut_attempts', [])),
                      'miss arbitration', timeout=5)
        miss_state = pose_from_entity(self.entity_state(SCENE['target_model']))
        self.summary['checks']['negative_misaligned'] = {
            'requested_pose_error_m': miss_error,
            'tcp_cutpoint_error_m': miss_cutpoint_error, 'x1': x1_at_miss,
            'fruit_displacement_m': float(np.linalg.norm(miss_state[0] - fruit_before[0])),
        }
        self.command_gripper('open')
        self.call(self.retreat, MoveToPose.Request(), 'retreat after miss', timeout=300)
        self.return_to_box_pose()

        # Positive case: only the exact blade/peduncle/slot contact can transfer ownership.
        feedback_start = len(self.feedback)
        attempt_start = len(json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
                            .get('cut_attempts', []))
        start = time.monotonic()
        positive_request, tcp_error = self.move_to(TARGET, TANGENT, expect_feedback=2)
        contact_tcp, contact_rotation = self.world_tcp_transform()
        fruit_at_grasp = pose_from_entity(self.entity_state(SCENE['target_model']))
        stem_count_before = json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
        self.command_gripper('close')
        self.wait_for(lambda: json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
                      .get('state') == 'HELD_BY_GRIPPER', 'fruit cut and grasp')
        self.call(self.retreat, MoveToPose.Request(), 'retract holding fruit', timeout=300)
        self.wait_for(lambda: True if json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
                      .get('state') == 'HELD_BY_GRIPPER' else None, 'held fruit follows TCP')
        fruit_after_retract = pose_from_entity(self.entity_state(SCENE['target_model']))
        tcp_after_retract, rotation_after_retract = self.world_tcp_transform()
        expected_fruit = (tcp_after_retract + rotation_after_retract.apply(
            contact_rotation.inv().apply(fruit_at_grasp[0] - contact_tcp)),
            rotation_after_retract * contact_rotation.inv() * fruit_at_grasp[1])
        follow_error = float(np.linalg.norm(fruit_after_retract[0] - expected_fruit[0]))
        follow_displacement = float(np.linalg.norm(fruit_after_retract[0] - fruit_at_grasp[0]))
        self.return_to_box_pose()
        self.command_gripper('open')
        self.wait_for(lambda: json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
                      .get('state') == 'RELEASED', 'dynamic release')
        settle_end = time.monotonic() + 4.0
        while time.monotonic() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        released_state = pose_from_entity(self.entity_state(SCENE['target_model']))
        basket_tf = self.transform('world', 'basket_link')
        basket_pos = np.array([basket_tf.translation.x, basket_tf.translation.y, basket_tf.translation.z])
        basket_rot = Rotation.from_quat([basket_tf.rotation.x, basket_tf.rotation.y,
                                         basket_tf.rotation.z, basket_tf.rotation.w])
        fruit_in_basket = basket_rot.inv().apply(released_state[0] - basket_pos)
        in_basket = bool(abs(fruit_in_basket[0]) < 0.16 and abs(fruit_in_basket[1]) < 0.21
                         and 0.10 < fruit_in_basket[2] < 0.32)
        after = self.camera_snapshot()
        camera = self.camera_removal_check(before, after, fruit_before[0])
        final_runtime = json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
        positive_events = final_runtime.get('cut_attempts', [])[attempt_start:]
        contacts_positive = final_runtime.get('main_stem_contacts', 0) - stem_count_before.get('main_stem_contacts', 0)
        self.summary['checks']['positive_harvest'] = {
            'test_result_2': any(result == 2 for _, result in self.feedback[feedback_start:]),
            'tcp_cutpoint_error_m': tcp_error,
            'tcp_final_error_m': float(np.linalg.norm(contact_tcp - TARGET)),
            'cut_success': any(item.get('event') == 'CUT_SUCCESS' for item in positive_events),
            'follow_error_m': follow_error,
            'follow_displacement_m': follow_displacement,
            'main_stem_contacts': contacts_positive,
            'release_position_in_basket_frame_m': fruit_in_basket.tolist(),
            'in_basket': in_basket,
            'camera_original_roi': camera,
        }
        self.summary['stage_times_s']['positive_close_to_release'] = time.monotonic() - start

        # Deliberate stem contact is tested only after the successful positive harvest.
        stem_start = len(self.feedback)
        stem_contacts_before = final_runtime.get('main_stem_contacts', 0)
        _, stem_tcp_error = self.move_to(STEM_POINT, STEM_TANGENT)
        self.wait_for(lambda: any(result == 3 for _, result in self.feedback[stem_start:]),
                      'main stem feedback result 3', timeout=5)
        final_runtime = json.loads((WORKSPACE / 'validation/phase3_runtime.json').read_text())
        self.summary['checks']['negative_main_stem'] = {
            'test_result_3': True, 'tcp_error_m': stem_tcp_error,
            'main_stem_contacts': final_runtime.get('main_stem_contacts', 0) - stem_contacts_before,
        }
        self.call(self.retreat, MoveToPose.Request(), 'retreat from main stem', timeout=300)
        self.set_home()

        checks = self.summary['checks']
        self.summary['passed'] = bool(
            self.summary.get('rail_station_accepted_within_tolerance', False)
            and checks['negative_misaligned']['requested_pose_error_m'] <= 0.005
            and checks['negative_misaligned']['tcp_cutpoint_error_m'] > 0.005
            and not checks['negative_misaligned']['x1']
            and checks['negative_misaligned']['fruit_displacement_m'] < 0.005
            and checks['positive_harvest']['test_result_2']
            and checks['positive_harvest']['cut_success']
            and checks['positive_harvest']['tcp_final_error_m'] <= 0.005
            and checks['positive_harvest']['follow_error_m'] <= 0.02
            and checks['positive_harvest']['follow_displacement_m'] > 0.02
            and checks['positive_harvest']['main_stem_contacts'] == 0
            and checks['positive_harvest']['in_basket']
            and checks['positive_harvest']['camera_original_roi'].get('visible', False)
            and checks['positive_harvest']['camera_original_roi'].get('rgb_changed_fraction', 0) > 0.10
            and checks['negative_main_stem']['test_result_3']
            and checks['negative_main_stem']['main_stem_contacts'] > 0)
        self.summary['passed'] = self.summary['passed'] and self.summary['checks']['positive_harvest'][
            'camera_original_roi'].get('depth_changed_fraction', 0) > 0.10
        (self.output / 'phase3_summary.json').write_text(json.dumps(self.summary, indent=2) + '\n')
        if not self.summary['passed']:
            raise RuntimeError('One or more Phase 3 acceptance checks failed')


def main():
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else WORKSPACE / 'validation'
    rclpy.init()
    node = Phase3Checks(output)
    try:
        node.run(float(sys.argv[2] if len(sys.argv) > 2 else
                       harvest_launch_defaults(load_configuration())['slide_initial']))
    except (Exception, KeyboardInterrupt) as error:
        node.summary['passed'] = False
        node.summary['error'] = str(error) or type(error).__name__
        (node.output / 'phase3_summary.json').write_text(json.dumps(node.summary, indent=2) + '\n')
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
