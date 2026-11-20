#!/usr/bin/env python3
"""Drive the running Gazebo robot and check its measured ROS and Gazebo state."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class VerifyRobot(Node):
    def __init__(self, joints, evidence):
        super().__init__('verify_phase1', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.joints = joints
        self.evidence = evidence
        self.q = {}
        self.velocity = {}
        self.samples = 0
        self.peak_velocity = {}
        self.robot_error = None
        self.cameras = {}
        self.create_subscription(JointState, '/joint_states', self.read_joints, qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_trajectory_controller/follow_joint_trajectory')
        self.slide = self.create_publisher(Float64MultiArray, '/slide_velocity_controller/commands', 10)
        self.gripper = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

    def read_joints(self, msg):
        self.q = dict(zip(msg.name, msg.position))
        self.velocity = dict(zip(msg.name, msg.velocity))
        self.samples += 1
        for name, velocity in self.velocity.items():
            self.peak_velocity[name] = max(self.peak_velocity.get(name, 0), abs(velocity))
        if not all(math.isfinite(value) for value in (*self.q.values(), *self.velocity.values())):
            self.robot_error = 'Nonfinite joint state'
        if abs(self.q.get('machine_slide', 0)) > 2.501:
            self.robot_error = 'Slide advanced beyond its limits'

    def until(self, predicate, label, timeout=25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            assert not self.robot_error, self.robot_error
            result = predicate()
            if result:
                return result
        raise RuntimeError(f'Timeout waiting for {label}. Last joint states: {self.q}')

    def dwell(self, seconds):
        deadline = self.get_clock().now().nanoseconds + round(seconds * 1e9)
        self.until(lambda: self.get_clock().now().nanoseconds >= deadline, 'simulation clock')

    def lookup(self, parent, child):
        try:
            return self.buffer.lookup_transform(parent, child, Time()).transform
        except TransformException:
            return None

    def command_arm(self, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=point, velocities=[0.0] * len(self.joints),
                                 time_from_start=Duration(sec=seconds))
            for seconds, point in points
        ]
        sent = self.arm.send_goal_async(goal)
        self.until(sent.done, 'arm goal acceptance')
        accepted = sent.result()
        assert accepted.accepted, 'Arm action rejected the goal'
        finished = accepted.get_result_async()
        self.until(finished.done, 'arm trajectory result', timeout=40)
        result = finished.result()
        assert result.status == GoalStatus.STATUS_SUCCEEDED, result
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, result
        return result.result.error_code

    def check_tcp(self, label, reference):
        q_expected = reference['q_rad']
        self.dwell(0.4)
        joint_error = max(abs(self.q[name] - q) for name, q in zip(self.joints, q_expected))
        assert joint_error < 0.003, (label, joint_error, self.q)
        tf = self.until(lambda: self.lookup('arm_base_link', 'tool_tcp'), 'arm-to-TCP TF')
        xyz = [tf.translation.x, tf.translation.y, tf.translation.z]
        xyzw = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
        result = {'xyz_m': xyz, 'xyzw': xyzw, 'max_joint_error_rad': joint_error}
        result['q_measured_rad'] = [self.q[name] for name in self.joints]
        for model in ('mujoco', 'aubo_i5'):
            other = reference[model]
            error_mm = 1000 * math.dist(xyz, other['xyz'])
            quat_dot = abs(sum(a*b for a, b in zip(xyzw, other['xyzw'])))
            result[f'{model}_position_error_mm'] = error_mm
            result[f'{model}_rotation_error_deg'] = math.degrees(2 * math.acos(min(1.0, quat_dot)))
            assert error_mm < 1.0, (label, model, error_mm)
        out = subprocess.run(
            ['timeout', '-s', 'INT', '3', 'ros2', 'run', 'tf2_ros', 'tf2_echo',
             'arm_base_link', 'tool_tcp'], capture_output=True, text=True, timeout=6)
        (self.evidence / f'tf2_echo_{label}.log').write_text(out.stdout + out.stderr)
        assert 'Translation:' in out.stdout, (label, out.stdout, out.stderr)
        print(f'{label} TCP errors: {result}', flush=True)
        return result

    def check_slide(self):
        start = self.q['machine_slide']
        assert abs(start) < 0.005, start
        try:
            self.slide.publish(Float64MultiArray(data=[0.2]))
            self.until(lambda: self.q['machine_slide'] >= start + 0.15, 'forward slide motion')
            forward = self.q['machine_slide']
            self.slide.publish(Float64MultiArray(data=[-0.2]))
            self.until(lambda: self.q['machine_slide'] <= start - 0.12, 'reverse slide motion')
            reverse = self.q['machine_slide']
        finally:
            self.slide.publish(Float64MultiArray(data=[0.0]))
        self.dwell(0.7)
        settled = self.q['machine_slide']
        self.dwell(0.7)
        drift = abs(self.q['machine_slide'] - settled)
        assert drift < 0.003 and abs(settled - reverse) < 0.01, (reverse, settled, drift)
        result = {'start_m': start, 'forward_m': forward, 'reverse_m': reverse,
                  'stopped_m': settled, 'hold_drift_m': drift}
        print(f'Slide: {result}', flush=True)
        return result

    def check_gripper(self):
        def measure(target):
            self.gripper.publish(Float64MultiArray(data=[target]))
            self.until(lambda: abs(self.q['gripper_joint'] - target) < 0.003, 'gripper target')
            self.dwell(0.25)
            # Classic reports mimic state names with a suffix.
            positions = [self.q[f'{side}_finger_joint_mimic'] for side in ('left', 'right')]
            for position, multiplier in zip(positions, (-0.020999, -0.020995)):
                assert abs(position - multiplier * target) < 0.0004, positions
            return {'gear_rad': self.q['gripper_joint'], 'finger_joint_m': positions}

        closed = measure(0.4)
        opened = measure(-0.45)
        for before, after in zip(closed['finger_joint_m'], opened['finger_joint_m']):
            assert after - before > 0.015, (closed, opened)
        result = {'closed': closed, 'reopened': opened}
        print(f'Gripper: {result}', flush=True)
        return result

    def check_cameras(self):
        def image_callback(name, stream, msg):
            if msg.data:
                self.cameras[(name, stream)] = (msg.header.frame_id, msg.width, msg.height, msg.encoding)

        subscriptions = []
        names = ('cam_left', 'cam_right', 'cam_hand')
        for name in names:
            for stream, topic in (('rgb', 'image_raw'), ('depth', 'depth/image_raw')):
                subscriptions.append(self.create_subscription(
                    Image, f'/sim/{name}/{topic}',
                    lambda msg, n=name, s=stream: image_callback(n, s, msg), qos_profile_sensor_data))
            subscriptions.append(self.create_subscription(
                CameraInfo, f'/sim/{name}/camera_info',
                lambda msg, n=name: self.cameras.update({(n, 'info'): msg}), qos_profile_sensor_data))
        try:
            self.until(lambda: len(self.cameras) == 9, 'three live RGB-D cameras and intrinsics')
            results = {}
            for name in names:
                expected_frame = f'{name}_color_optical_frame'
                frame, width, height, encoding = self.cameras[(name, 'rgb')]
                assert (frame, width, height, encoding) == (expected_frame, 848, 480, 'rgb8')
                assert self.cameras[(name, 'depth')] == (expected_frame, width, height, '32FC1')
                info = self.cameras[(name, 'info')]
                assert info.header.frame_id == expected_frame and info.k[0] > 0 and info.k[4] > 0
                self.until(lambda n=name: self.lookup('world', f'{n}_color_optical_frame'), name)
                results[name] = {'frame': frame, 'width': width, 'height': height,
                                 'rgb': encoding, 'depth': '32FC1', 'fx_px': info.k[0]}
            print(f'Cameras: {results}', flush=True)
            return results
        finally:
            for subscription in subscriptions:
                self.destroy_subscription(subscription)


def gazebo_anchor(world_name):
    pose = subprocess.check_output(
        ['gz', 'model', '-w', world_name, '-m', 'tomato_picker', '-p'],
        text=True, timeout=5)
    values = [float(component) for component in pose.split()]
    assert len(values) == 6 and all(math.isfinite(component) for component in values), values
    return values


def main():
    evidence = Path(sys.argv[1])
    if len(sys.argv) > 2:
        world_name = sys.argv[2]
    else:
        from simulation_config import load_configuration, read_world
        world_name = read_world(load_configuration()['world'])[1].get('name')
    references = json.loads((evidence / 'fk_reference.json').read_text())
    poses = references['poses']
    rclpy.init()
    node = VerifyRobot(references['joints'], evidence)
    try:
        assert node.arm.wait_for_server(timeout_sec=12)
        required = set(node.joints) | {'machine_slide', 'gripper_joint',
                                     'left_finger_joint_mimic', 'right_finger_joint_mimic'}
        node.until(lambda: required <= node.q.keys()
                   and node.slide.get_subscription_count()
                   and node.gripper.get_subscription_count(), 'actuator state and command subscriptions')
        anchor_start = gazebo_anchor(world_name)
        result = {'cameras': node.check_cameras()}
        result['slide'] = node.check_slide()
        zero = poses['zero']['q_rad']
        node.command_arm([(2, zero)])
        result['fk'] = {'zero': node.check_tcp('zero', poses['zero'])}
        node.command_arm([(2, poses['pose_a']['q_rad'])])
        result['fk']['pose_a'] = node.check_tcp('pose_a', poses['pose_a'])
        node.command_arm([(1, zero), (3, poses['pose_b']['q_rad'])])
        result['fk']['pose_b'] = node.check_tcp('pose_b', poses['pose_b'])
        result['gripper'] = node.check_gripper()
        node.command_arm([(2, zero)])
        node.dwell(0.5)
        anchor_end = gazebo_anchor(world_name)
        assert max(abs(a - b) for a, b in zip(anchor_end, anchor_start)) < 0.001, (anchor_start, anchor_end)
        assert max(abs(node.q[name]) for name in node.joints) < 0.003, node.q
        assert all(speed < 3.2 for joint, speed in node.peak_velocity.items() if joint in node.joints)
        assert node.peak_velocity['machine_slide'] < 0.5
        result.update(anchor_start_xyz_rpy=anchor_start, anchor_end_xyz_rpy=anchor_end,
                      joint_state_samples=node.samples, peak_joint_velocity=node.peak_velocity)
        (evidence / 'motion_results.json').write_text(json.dumps(result, indent=2) + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
