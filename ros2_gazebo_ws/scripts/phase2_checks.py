#!/usr/bin/env python3
"""Verify camera scale against Gazebo geometry and call the real service contracts."""
import json
from pathlib import Path
import re
import subprocess
import sys
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, UInt8
from tf2_ros import Buffer, TransformException, TransformListener
from xc_interfaces.msg import AGVStatus, ArmStatus, MidpointReached

from tomato_sim_adapters.arm_kinematics import ArmKinematics, JOINTS, OEM_HOME

TARGET_JOINTS = np.deg2rad([10.0, 15.0, 135.0, 117.0, 85.0, 0.0])


class Checks(Node):
    def __init__(self, evidence):
        super().__init__('verify_phase2', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')
        self.sim_input = self.create_publisher(Bool, '/sim/harvest/input_x1', 10)
        self.options = {'evidence': evidence}
        self.frames = {}
        self.q = {}
        self.arm_status = None
        self.agv = None
        self.sensors = []
        self.midpoints = []
        self.approaches = []
        self.cli_log = []
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(ArmStatus, '/arm_status', self.read_arm, 10)
        self.create_subscription(AGVStatus, '/agv_status', self.read_agv, 10)
        self.create_subscription(MidpointReached, '/midpoint_reached', self.read_mid, 10)
        self.create_subscription(UInt8, '/gripper/input_x1_status', self.read_x1, 10)
        self.create_subscription(Bool, '/sim/harvest/approaching', self.read_approach, 10)

    def joints(self, msg):
        self.q = dict(zip(msg.name, msg.position))

    def read_arm(self, msg):
        self.arm_status = msg

    def read_agv(self, msg):
        self.agv = msg

    def read_mid(self, msg):
        self.midpoints.append((self.get_clock().now().nanoseconds, msg.reached))
        if msg.reached:
            self.sim_input.publish(Bool(data=True))

    def read_x1(self, msg):
        self.sensors.append((self.get_clock().now().nanoseconds, msg.data))

    def read_approach(self, msg):
        self.approaches.append((self.get_clock().now().nanoseconds, msg.data))

    def wait_for(self, predicate, description, timeout=25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            value = predicate()
            if value is not None and value is not False:
                return value
        raise RuntimeError(f'Timed out waiting for {description}. Slide={self.q.get("machine_slide")}, AGV={self.agv}')

    def robot_pose(self, parent, child):
        def lookup():
            try:
                return self.tf.lookup_transform(parent, child, Time()).transform
            except TransformException:
                return None
        transform = self.wait_for(lookup, f'{parent} to {child} TF')
        t, q = transform.translation, transform.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])

    def cli(self, *args, expect=None, timeout=35):
        with subprocess.Popen(['ros2', *args], text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT) as process:
            try:
                self.wait_for(lambda: process.poll() is not None, ' '.join(args), timeout)
                output = process.communicate()[0]
            finally:
                if process.poll() is None:
                    process.kill()
            self.cli_log.append('$ ros2 ' + ' '.join(args) + '\n' + output)
            (self.options['evidence'] / 'phase2_cli.log').write_text('\n'.join(self.cli_log))
            assert process.returncode == 0 and (expect is None or re.search(expect, output)), output
            return output

    def capture(self, name, stream, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        group = self.frames.setdefault((name, msg.header.frame_id, stamp), {})
        group[stream] = msg
        for key in tuple(self.frames):
            if stamp - key[2] > 350_000_000:
                del self.frames[key]

    def camera(self, prefix, frame, units, color_topic):
        subscriptions = [
            self.create_subscription(
                message_type, prefix + topic,
                lambda msg, label=stream: self.capture(prefix, label, msg), qos_profile_sensor_data)
            for stream, message_type, topic in (
                ('color', Image, color_topic), ('depth', Image, '/aligned_depth_to_color/image_raw'),
                ('info', CameraInfo, '/color/camera_info'))
        ]
        label = 'phase2_plate_' + frame
        translation, rotation = self.robot_pose('world', frame)
        front_center = np.array([.023, -.015, .800])
        box_center = translation + rotation.apply(front_center + [0, 0, .010])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = box_center.tolist()
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = (
            rotation.as_quat().tolist())
        sdformat = f"""
        <sdf version='1.6'><model name='{label}'><static>true</static><link name='plate'>
          <visual name='plate'><geometry><box><size>0.16 0.16 0.02</size></box></geometry>
             <material><ambient>0.9 0.8 0.2 1</ambient><diffuse>0.9 0.8 0.2 1</diffuse></material>
          </visual>
        </link></model></sdf>"""
        assert self.spawn.wait_for_service(timeout_sec=10)
        future = self.spawn.call_async(SpawnEntity.Request(name=label, xml=sdformat, initial_pose=pose))
        self.wait_for(future.done, 'calibration board spawn')
        assert future.result().success, future.result()
        threshold = self.get_clock().now().nanoseconds
        try:
            def measurement():
                for (source, camera_frame, stamp), group in reversed(list(self.frames.items())):
                    if source != prefix or camera_frame != frame or stamp <= threshold or len(group) != 3:
                        continue
                    depth = group['depth']
                    info = group['info']
                    assert depth.encoding == '16UC1' and not depth.is_bigendian
                    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
                    u, v = (round(cx + fx*front_center[0]/front_center[2]),
                            round(cy + fy*front_center[1]/front_center[2]))
                    raw = np.ndarray((depth.height, depth.width), dtype='<u2',
                                     buffer=depth.data, strides=(depth.step, 2))
                    z = raw[v, u] / units
                    if abs(z - .8) > .01:
                        continue
                    point = np.array([(u-cx)*z/fx, (v-cy)*z/fy, z])
                    point_world = translation + rotation.apply(point)
                    true_world = translation + rotation.apply(front_center)
                    error = float(np.linalg.norm(point_world - true_world) * 1000)
                    assert error < 3.0, (frame, point, error)
                    return {'frame': frame, 'pixel': [u, v], 'count': int(raw[v, u]),
                            'depth_m': z, 'error_mm': error,
                            'gazebo_point_world': true_world.tolist(),
                            'backprojected_world': point_world.tolist(),
                            'stamp_ns': stamp}
                return None

            result = self.wait_for(measurement, f'0.800 m calibration board in {frame}')
            print(f'Camera {prefix}: {result}', flush=True)
            return result
        finally:
            future = self.delete.call_async(DeleteEntity.Request(name=label))
            self.wait_for(future.done, 'calibration board removal')
            assert future.result().success
            for subscription in subscriptions:
                self.destroy_subscription(subscription)

    def command(self, endpoint, service, request, expect):
        return self.cli('service', 'call', endpoint, 'xc_interfaces/srv/' + service,
                        json.dumps(request), expect=expect)

    def q_error(self, expected):
        return float(np.max(np.abs([self.q[joint] - q for joint, q in zip(JOINTS, expected)])))

    def pose_error(self, position):
        self.wait_for(lambda: self.arm_status is not None, 'AUBO status')
        p = self.arm_status.position
        return float(np.linalg.norm(np.array([p.x, p.y, p.z]) - position) * 1000)

    def exercise_agv(self):
        start = self.q['machine_slide']
        response = r'control_cmd_success=True, status_cmd_success=True'
        self.command('/control_agv', 'ControlAGV', {'linear_speed': 100, 'lift_magnetic_sensor': True}, response)
        self.wait_for(lambda: self.q['machine_slide'] - start >= .05 and self.agv
                      and abs(self.agv.left_wheel_speed - 100) <= 2, '100 mm/s measured AGV motion')
        self.command('/control_agv', 'ControlAGV', {'linear_speed': 0, 'lift_magnetic_sensor': True}, response)
        self.wait_for(lambda: self.agv and abs(self.agv.left_wheel_speed) <= 2, 'AGV stop')
        stopped = self.q['machine_slide']
        self.cli('param', 'set', '/agv_always_go_straght_node', 'is_continue_run', 'true',
                 expect='Set parameter successful')
        self.wait_for(lambda: self.q['machine_slide'] - stopped >= .05, 'AGV autonomous motion')
        self.cli('param', 'set', '/agv_always_go_straght_node', 'is_continue_run', 'false',
                 expect='Set parameter successful')
        self.wait_for(lambda: self.agv and abs(self.agv.left_wheel_speed) <= 2, 'autonomous AGV stop')
        parameter_stop = self.q['machine_slide']
        card_positions = {}
        for speed, card in ((500, 13), (-500, 23)):
            self.command('/control_agv', 'ControlAGV', {'linear_speed': speed}, response)
            self.wait_for(lambda: self.agv and self.agv.white_card_id == card
                          and abs(self.agv.left_wheel_speed) <= 2, f'RFID {card} and rail stop')
            card_positions[str(card)] = self.q['machine_slide']
        assert 2.4 <= card_positions['13'] < 2.45 and -1.90 < card_positions['23'] <= -1.85
        self.command('/control_agv', 'ControlAGV', {'linear_speed': 400}, response)
        self.wait_for(lambda: self.q['machine_slide'] >= -0.30 and self.agv
                      and self.agv.white_card_id == 0, 'RFID clear and parking approach')
        self.command('/control_agv', 'ControlAGV', {'linear_speed': 0}, response)
        self.wait_for(lambda: self.agv and abs(self.agv.left_wheel_speed) <= 2, 'parking stop')
        return {'initial_m': start, 'service_stop_m': stopped, 'parameter_stop_m': parameter_stop,
                'rfid_stop_positions_m': card_positions, 'parking_position_m': self.q['machine_slide']}

    def exercise_arm(self):
        kin = ArmKinematics()
        goal = kin.fk_tcp(TARGET_JOINTS)
        pos = np.array(list(goal.p))
        request = dict(zip(('x', 'y', 'z', 'rx', 'ry', 'rz'), [*pos, *goal.M.GetRPY()]))
        request.update(mid2target_angle=0.0, mid_to_target=False, start2mid=True)
        midpoint = pos + [0.0, 0.1, 0.0]
        self.command('/move_joints', 'MoveJoints', {'joints': OEM_HOME.tolist()}, r'result=0\)')
        home_error = self.q_error(OEM_HOME)
        assert home_error < 0.003, home_error
        self.command('/move_to_pose', 'MoveToPose', request, r'result=0\)')
        midpoint_error = self.pose_error(midpoint)
        assert midpoint_error < 3.0 and self.midpoints[-1][1], (midpoint_error, self.midpoints)
        assert not any(state for _, state in self.approaches)
        request.update(mid_to_target=True, start2mid=False)
        self.command('/move_to_pose', 'MoveToPose', request, r'result=0\)')
        target_error = self.pose_error(pos)
        assert target_error < 3.0, target_error
        reached = next(stamp for stamp, state in self.midpoints if state)
        approach = next(stamp for stamp, state in self.approaches if state)
        sensor = next(stamp for stamp, state in self.sensors if state)
        assert reached < approach <= sensor, (reached, approach, sensor)
        self.command('/gripper/control', 'GripperControl', {'command': 'close'},
                     r'success=True, current_position=270[012]')
        self.command('/target_return_midpoint', 'MoveToPose', {}, r'result=0\)')
        retreat_error = self.pose_error(midpoint)
        assert retreat_error < 3.0 and not self.midpoints[-1][1], retreat_error
        self.command('/return_to_box', 'MoveToPose', {}, r'result=0\)')
        basket, _ = self.robot_pose('basket_link', 'tool_tcp')
        assert abs(basket[0]) < .03 and abs(basket[1]) < .03 and basket[2] < -.15, basket
        self.command('/gripper/control', 'GripperControl', {'command': 'open'},
                     r'success=True, current_position=60[012]')
        self.command('/move_joints', 'MoveJoints', {'joints': OEM_HOME.tolist()}, r'result=0\)')
        reset_error = self.q_error(OEM_HOME)
        assert reset_error < .003 and self.sensors[-1][1] == 0
        request.update(mid_to_target=False, start2mid=False)
        self.command('/move_to_pose', 'MoveToPose', request, r'result=0\)')
        combined_error = self.pose_error(pos)
        assert combined_error < 3.0, combined_error
        self.command('/target_return_midpoint', 'MoveToPose', {}, r'result=0\)')
        self.command('/move_joints', 'MoveJoints', {'joints': OEM_HOME.tolist()}, r'result=0\)')
        return {'home_joint_error_rad': home_error, 'midpoint_error_mm': midpoint_error,
                'target_error_mm': target_error, 'retreat_error_mm': retreat_error,
                'basket_to_tcp_m_in_basket_frame': basket.tolist(), 'reset_joint_error_rad': reset_error,
                'combined_approach_error_mm': combined_error,
                'midpoint_stamp_ns': reached, 'approach_stamp_ns': approach, 'x1_stamp_ns': sensor}


def main():
    evidence = Path(sys.argv[1])
    rclpy.init()
    checks = Checks(evidence)
    try:
        checks.wait_for(lambda: checks.arm_status and checks.agv and set(JOINTS) <= checks.q.keys()
                        and checks.sim_input.get_subscription_count() > 0, 'robot and adapters', 40)
        results = {'cameras': {}}
        results['cameras']['left'] = checks.camera('/camera', 'cam_left_color_optical_frame',
                                                   4000, '/color/image_raw')
        checks.cli('param', 'set', '/sim_camera_mux_relay', 'active_side', 'right',
                   expect='Set parameter successful')
        results['cameras']['right'] = checks.camera('/camera', 'cam_right_color_optical_frame',
                                                    4000, '/color/image_raw')
        results['cameras']['hand'] = checks.camera('/cam_hand', 'cam_hand_color_optical_frame',
                                                   1000, '/color/image_rect_raw')
        (evidence / 'phase2_cameras.json').write_text(json.dumps(results, indent=2) + '\n')
        results['agv'] = checks.exercise_agv()
        print(f'AGV: {results["agv"]}', flush=True)
        results['arm'] = checks.exercise_arm()
        print(f'Arm and gripper: {results["arm"]}', flush=True)
        (evidence / 'phase2_summary.json').write_text(json.dumps(results, indent=2) + '\n')
    finally:
        checks.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
