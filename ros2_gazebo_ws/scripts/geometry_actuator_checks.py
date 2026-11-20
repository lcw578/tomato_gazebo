#!/usr/bin/env python3
"""Measure basket return, RGB-D touch and gripper tracking through ROS services."""
import argparse
import json
from pathlib import Path

import cv2
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState, SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import numpy as np
import rclpy
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformException
from xc_interfaces.srv import GripperControl, MoveToPose

from phase2_checks import Checks
from rgbd_sphere import measure_sphere
from tomato_sim_adapters.arm_kinematics import ArmKinematics, JOINTS


class GeometryChecks(Checks):
    def __init__(self, output):
        super().__init__(output)
        self.output = output
        self.results = {}
        self.park_origin = None
        self.park_samples = []
        self.return_client = self.create_client(MoveToPose, '/return_to_box')
        self.gripper_client = self.create_client(GripperControl, '/gripper/control')
        self.entity_client = self.create_client(GetEntityState, '/get_entity_state')
        self.move_client = self.create_client(MoveToPose, '/move_to_pose')

    def joints(self, message):
        super().joints(message)
        if self.park_origin is not None and 'machine_slide' in self.q:
            self.park_samples.append(self.q['machine_slide'])

    def begin_parking_measurement(self):
        self.park_origin = self.q['machine_slide']
        self.park_samples = [self.park_origin]

    def save_parking_measurement(self):
        samples = np.array(self.park_samples)
        self.results['parking'] = {
            'initial_slide_m': self.park_origin,
            'minimum_slide_m': float(samples.min()),
            'maximum_slide_m': float(samples.max()),
            'max_displacement_mm': float(np.max(np.abs(samples - self.park_origin)) * 1000),
            'peak_to_peak_mm': float(np.ptp(samples) * 1000),
            'sample_count': len(samples),
            'world_target_compensation_enabled': False,
        }
        self.save()

    def parking(self):
        from tomato_sim_adapters.arm_kinematics import OEM_HOME
        from xc_interfaces.srv import MoveJoints
        from phase2_checks import TARGET_JOINTS

        self.begin_parking_measurement()
        client = self.create_client(MoveJoints, '/move_joints')
        try:
            for target in (OEM_HOME, TARGET_JOINTS, OEM_HOME):
                response = self.call(client, MoveJoints.Request(joints=target.tolist()))
                assert response.result == 0, response
                self.dwell(0.5)
        finally:
            self.save_parking_measurement()
        assert self.results['parking']['max_displacement_mm'] < 0.5, self.results['parking']

    def rail(self):
        from xc_interfaces.srv import ControlAGV

        client = self.create_client(ControlAGV, '/control_agv')
        samples = []
        self.results['rail'] = samples
        try:
            for speed in (100, -100):
                self.park_origin = None
                response = self.call(client, ControlAGV.Request(linear_speed=speed))
                assert response.control_cmd_success, response
                self.dwell(0.25)
                start = self.q['machine_slide']
                start_time = self.get_clock().now().nanoseconds
                self.dwell(2.0)
                duration = (self.get_clock().now().nanoseconds - start_time) / 1e9
                displacement = self.q['machine_slide'] - start
                response = self.call(client, ControlAGV.Request(linear_speed=0))
                assert response.control_cmd_success, response
                self.dwell(0.5)
                self.parking()
                sample = {'command_mm_s': speed, 'duration_sim_s': duration,
                          'displacement_m': displacement,
                          'measured_speed_mm_s': displacement / duration * 1000,
                          'parking_after_stop': dict(self.results['parking'])}
                samples.append(sample)
                self.save()
                assert abs(sample['measured_speed_mm_s'] - speed) < 5.0, sample
        finally:
            self.call(client, ControlAGV.Request(linear_speed=0))

    def call(self, client, request, timeout=300):
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError(f'Missing service {client.srv_name}')
        future = client.call_async(request)
        self.wait_for(future.done, client.srv_name, timeout)
        return future.result()

    def dwell(self, seconds):
        end = self.get_clock().now().nanoseconds + round(seconds * 1e9)
        self.wait_for(lambda: self.get_clock().now().nanoseconds >= end, 'simulation settling', 60)

    def save(self):
        (self.output / 'measurements.json').write_text(json.dumps(self.results, indent=2) + '\n')

    def basket(self):
        self.begin_parking_measurement()
        response = self.call(self.return_client, MoveToPose.Request())
        self.results['return_service_result'] = response.result
        self.save()
        if response.result:
            raise AssertionError(f'Return service failed with {response.result}')
        self.dwell(0.5)
        xyz, rotation = self.robot_pose('basket_link', 'tool_tcp')
        q = [self.q[name] for name in JOINTS]
        base_xyz, base_rotation = self.robot_pose('basket_link', 'arm_base_link')
        fk = ArmKinematics().fk_tcp(q)
        fk_xyz = base_xyz + base_rotation.apply(list(fk.p))
        release_z = min(-0.22, base_xyz[2] - 0.20)
        # The CAD crate interior is 0.30 x 0.41 m. Reserve a 35 mm fruit radius.
        clearance = [0.15 - abs(xyz[0]) - 0.035, 0.205 - abs(xyz[1]) - 0.035]
        self.results['basket'] = {
            'joints_rad': q, 'tcp_in_basket_m': xyz.tolist(),
            'arm_base_in_basket_m': base_xyz.tolist(),
            'arm_base_rpy_in_basket_rad': base_rotation.as_euler('xyz').tolist(),
            'tcp_in_arm_base_m': list(fk.p),
            'tcp_rpy_in_arm_base_rad': list(fk.M.GetRPY()),
            'release_target_in_basket_m': [0, 0, float(release_z)],
            'tcp_rpy_in_basket_rad': rotation.as_euler('xyz').tolist(),
            'target_error_mm': float(np.linalg.norm(xyz - [0, 0, release_z]) * 1000),
            'fk_tf_error_mm': float(np.linalg.norm(xyz - fk_xyz) * 1000),
            'centered_35mm_fruit_edge_clearance_m': clearance,
            'tcp_above_basket_opening': bool(min(clearance) > 0 and xyz[2] < 0),
        }
        self.save()
        assert self.results['basket']['target_error_mm'] < 3.0, self.results['basket']
        assert self.results['basket']['tcp_above_basket_opening'], self.results['basket']
        self.save_parking_measurement()
        assert self.results['parking']['max_displacement_mm'] < 0.5, self.results['parking']

    def gripper(self, cycles):
        samples = []
        self.results['gripper'] = samples
        for cycle in range(cycles):
            for command, target in (('close', 0.4), ('open', -0.45)):
                response = self.call(self.gripper_client, GripperControl.Request(command=command), 30)
                self.dwell(0.5)
                fingers = [self.q[f'{side}_finger_joint_mimic'] for side in ('left', 'right')]
                expected = np.array([-0.020999, -0.020995]) * target
                sample = {
                    'cycle': cycle, 'command': command, 'service_success': response.success,
                    'gear_rad': self.q['gripper_joint'], 'fingers_m': fingers,
                    'target_residual_mm': ((np.array(fingers) - expected) * 1000).tolist(),
                    'mimic_residual_mm': ((np.array(fingers) - np.array([-0.020999, -0.020995])
                                           * self.q['gripper_joint']) * 1000).tolist(),
                }
                samples.append(sample)
                self.save()
        print(json.dumps(samples, indent=2), flush=True)
        assert all(s['service_success'] and max(map(abs, s['target_residual_mm'])) < 0.4
                   for s in samples), samples

    def sphere(self, camera):
        # A small ball fits the cutter's access gap around the cutting TCP.
        radius = 0.004
        # Registered depth is quantized to 1 mm. A 0.5 mm approach margin
        # makes contact observable while keeping penetration below that scale.
        contact_margin = 0.0005
        name = 'rgbd_calibration_sphere'
        frame = f'cam_{camera}_color_optical_frame'
        self.begin_parking_measurement()
        # A static sphere also provides external collision geometry for contact checks.
        sdf = f'''<sdf version="1.6"><model name="{name}"><static>true</static>
          <link name="ball"><visual name="ball"><geometry><sphere><radius>{radius}</radius>
          </sphere></geometry><material><ambient>1 0 1 1</ambient><diffuse>1 0 1 1</diffuse>
          </material></visual><collision name="ball"><geometry><sphere><radius>{radius}</radius>
          </sphere></geometry><surface><contact><ode><kp>10000</kp><kd>1</kd>
          <max_vel>0.01</max_vel><min_depth>0.00001</min_depth></ode></contact></surface>
          </collision></link></model></sdf>'''
        from tomato_sim_adapters.arm_kinematics import OEM_HOME
        from xc_interfaces.srv import MoveJoints

        # Start from the same initialization joints as the hardware controller.
        joints_client = self.create_client(MoveJoints, '/move_joints')
        response = self.call(joints_client, MoveJoints.Request(joints=OEM_HOME.tolist()))
        assert response.result == 0, response
        response = self.call(self.gripper_client, GripperControl.Request(command='close'), 30)
        assert response.success, response
        camera_xyz, camera_rotation = self.robot_pose('world', frame)
        # Use reachable, visible fixtures on each side of the installed arm.
        optical_position = {'right': [0.02, -0.02, 0.35],
                            'left': [-0.06, -0.12, 0.42],
                            'hand': [0.0, 0.0, 0.18]}[camera]
        truth = camera_xyz + camera_rotation.apply(optical_position)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = truth.tolist()
        prefix = '/cam_hand' if camera == 'hand' else '/camera'
        color_topic = 'image_rect_raw' if camera == 'hand' else 'image_raw'
        if camera != 'hand':
            self.cli('param', 'set', '/sim_camera_mux_relay', 'active_side', camera, expect='successful')
            parameter_client = self.create_client(GetParameters, '/sim_camera_mux_relay/get_parameters')
            parameters = self.call(parameter_client, GetParameters.Request(
                names=['remote_depth_units_per_metre']))
            depth_units = parameters.values[0].integer_value
            if depth_units <= 0:
                raise ValueError('Camera relay depth scale must be positive')
        else:
            depth_units = 1000
        subscriptions = [self.create_subscription(
            kind, topic, lambda msg, label=label: self.capture(prefix, label, msg),
            qos_profile_sensor_data) for kind, topic, label in (
                (Image, f'{prefix}/color/{color_topic}', 'color'),
                (Image, f'{prefix}/aligned_depth_to_color/image_raw', 'depth'),
                (CameraInfo, f'{prefix}/color/camera_info', 'info'))]
        ball_contacts = set()

        def capture_ball_contact(message):
            for contact in message.states:
                if name in contact.collision1_name or name in contact.collision2_name:
                    ball_contacts.add((contact.collision1_name, contact.collision2_name))

        subscriptions.append(self.create_subscription(
            ContactsState, '/sim/harvest/blade_contacts', capture_ball_contact, qos_profile_sensor_data))
        try:
            response = self.call(self.spawn, SpawnEntity.Request(name=name, xml=sdf, initial_pose=pose))
            assert response.success, response.status_message
            after = self.get_clock().now().nanoseconds

            def detect():
                for (_, source, stamp), group in reversed(list(self.frames.items())):
                    if source != frame or stamp <= after or len(group) != 3:
                        continue
                    rgb_msg, depth_msg, info = group['color'], group['depth'], group['info']
                    color = np.ndarray((rgb_msg.height, rgb_msg.width, 3), dtype=np.uint8,
                                       buffer=rgb_msg.data, strides=(rgb_msg.step, 3, 1)).copy()
                    if rgb_msg.encoding == 'bgr8':
                        color = color[:, :, ::-1].copy()
                    dtype = '<u2' if depth_msg.encoding == '16UC1' else '<f4'
                    depth = np.ndarray((depth_msg.height, depth_msg.width), dtype=dtype,
                                       buffer=depth_msg.data, strides=(depth_msg.step, np.dtype(dtype).itemsize)).copy()
                    units = depth_units if depth_msg.encoding == '16UC1' else 1
                    try:
                        measurement = measure_sphere(color, depth, np.array(info.k).reshape(3, 3), radius, units)
                    except ValueError:
                        continue
                    return measurement, stamp, color, depth
                return None

            measurement, stamp, color, depth = self.wait_for(detect, 'RGB-D sphere detection', 60)
            capture_time = Time(nanoseconds=stamp)

            def exposure_transform(parent, child):
                def lookup():
                    try:
                        return self.tf.lookup_transform(parent, child, capture_time).transform
                    except TransformException:
                        return None
                transform = self.wait_for(lookup, f'{parent} to {child} at exposure', 10)
                p, q = transform.translation, transform.rotation
                return np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])

            camera_in_base, camera_rotation_in_base = exposure_transform('arm_base_link', frame)
            matrix = np.eye(4)
            matrix[:3, :3] = camera_rotation_in_base.as_matrix()
            matrix[:3, 3] = camera_in_base
            np.savetxt(self.output / 'hand_eye_matrix.csv', matrix, delimiter=',')
            optical_center = measurement.pop('center_camera_m')
            direction = optical_center / np.linalg.norm(optical_center)
            optical_touch = optical_center - (radius - contact_margin) * direction
            target = (matrix @ np.r_[optical_touch, 1])[:3]
            measured_center = (matrix @ np.r_[optical_center, 1])[:3]
            base_xyz, base_rotation = exposure_transform('world', 'arm_base_link')
            center_world = base_xyz + base_rotation.apply(measured_center)
            self.results['sphere'] = {
                **measurement, 'image_stamp_ns': stamp, 'camera_frame': frame,
                'depth_units_per_metre': depth_units,
                'sphere_radius_m': radius, 'true_center_world_m': truth.tolist(),
                'contact_approach_margin_m': contact_margin,
                'measured_center_camera_m': optical_center.tolist(),
                'measured_center_world_m': center_world.tolist(),
                'center_error_mm': float(np.linalg.norm(center_world - truth) * 1000),
                'target_touch_base_m': target.tolist(), 'hand_eye_matrix': matrix.tolist(),
            }
            cv2.imwrite(str(self.output / 'sphere_rgb.png'), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
            np.savez_compressed(self.output / 'sphere_depth.npz', depth=depth)
            self.save()
            # Expose the blade edge to the ball along local -Y.
            approach = matrix[:3, :3] @ direction
            tool_y = -approach
            tool_x = np.cross(tool_y, [0, 0, -1])
            tool_x /= np.linalg.norm(tool_x)
            tool_z = np.cross(tool_x, tool_y)
            rpy = Rotation.from_matrix(np.column_stack((tool_x, tool_y, tool_z))).as_euler('xyz')
            request = MoveToPose.Request(x=float(target[0]), y=float(target[1]), z=float(target[2]),
                                         rx=float(rpy[0]), ry=float(rpy[1]), rz=float(rpy[2]), start2mid=True)
            response = self.call(self.move_client, request)
            assert response.result == 0, response
            request.start2mid = False
            request.mid_to_target = True
            commanded_world = base_xyz + base_rotation.apply(target)
            # Submit the exposure-time arm-base target once. Parking must hold
            # the base while the arm moves, including the midpoint trajectory.
            response = self.call(self.move_client, request)
            assert response.result == 0, response
            self.dwell(0.5)
            current_base, _ = self.robot_pose('world', 'arm_base_link')
            actual, _ = self.robot_pose('world', 'tool_tcp')
            physical = self.call(self.entity_client, GetEntityState.Request(
                name='tomato_picker::wrist3_Link', reference_frame='world'))
            assert physical.success, physical.status_message
            wrist = physical.state.pose
            wrist_rotation = Rotation.from_quat([wrist.orientation.x, wrist.orientation.y,
                                                wrist.orientation.z, wrist.orientation.w])
            physical_tcp = (np.array([wrist.position.x, wrist.position.y, wrist.position.z])
                            + wrist_rotation.apply(list(ArmKinematics().wrist_to_tcp.p)))
            self.results['sphere'].update({
                'actual_tcp_world_m': actual.tolist(), 'physical_tcp_world_m': physical_tcp.tolist(),
                'tcp_command_error_mm': float(np.linalg.norm(actual - commanded_world) * 1000),
                'tcp_sphere_surface_error_mm': float(abs(np.linalg.norm(physical_tcp - truth) - radius) * 1000),
                'physical_tf_error_mm': float(np.linalg.norm(actual - physical_tcp) * 1000),
                'world_target_compensation_enabled': False,
                'target_reexpressions_after_midpoint': [],
                'base_translation_during_touch_m': (current_base - base_xyz).tolist(),
                'blade_ball_contact_detected': bool(ball_contacts),
                'blade_ball_collision_pairs': sorted(ball_contacts),
            })
            self.save()
            self.save_parking_measurement()
            assert self.results['parking']['max_displacement_mm'] < 0.5, self.results['parking']
            assert self.results['sphere']['center_error_mm'] < 3.0, self.results['sphere']
            assert self.results['sphere']['tcp_sphere_surface_error_mm'] < 3.0, self.results['sphere']
            assert self.results['sphere']['tcp_command_error_mm'] < 3.0, self.results['sphere']
            assert ball_contacts, 'The blade did not physically contact the calibration sphere'
        finally:
            self.save_parking_measurement()
            self.call(self.delete, DeleteEntity.Request(name=name), 15)
            for subscription in subscriptions:
                self.destroy_subscription(subscription)

    def external_contact(self):
        name = 'blade_external_contact_probe'
        response = self.call(self.gripper_client, GripperControl.Request(command='close'), 30)
        assert response.success, response
        self.dwell(0.25)
        state = self.call(self.entity_client, GetEntityState.Request(
            name='tomato_picker::left_finger_link', reference_frame='world'))
        assert state.success, state.status_message
        mesh_path = (Path(__file__).resolve().parents[1]
                     / 'src/tomato_sim_description/meshes/gripper/part_005_solid_005.stl')
        triangles = np.frombuffer(mesh_path.read_bytes(), offset=84, dtype=np.dtype([
            ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
        vertices = triangles['vertices'].reshape(-1, 3).astype(float) * 0.001
        local_center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
        local_center = (Rotation.from_euler('xyz', [1.5710562, -0.000000071387875, -0.00027471871])
                        .apply(local_center) + [0.042380645, -0.079623834, -0.06351737])
        finger = state.state.pose
        finger_rotation = Rotation.from_quat([finger.orientation.x, finger.orientation.y,
                                             finger.orientation.z, finger.orientation.w])
        point = (np.array([finger.position.x, finger.position.y, finger.position.z])
                 + finger_rotation.apply(local_center))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point.tolist()
        contacts = set()

        def capture(message):
            for contact in message.states:
                if name in contact.collision1_name or name in contact.collision2_name:
                    contacts.add((contact.collision1_name, contact.collision2_name))

        subscription = self.create_subscription(ContactsState, '/sim/harvest/blade_contacts',
                                                capture, qos_profile_sensor_data)
        sdf = f'''<sdf version="1.6"><model name="{name}"><static>true</static><link name="probe">
          <collision name="probe"><geometry><box><size>0.006 0.006 0.006</size></box></geometry>
          </collision></link></model></sdf>'''
        try:
            result = self.call(self.spawn, SpawnEntity.Request(name=name, xml=sdf, initial_pose=pose))
            assert result.success, result.status_message
            self.wait_for(lambda: bool(contacts), 'external blade contact', 20)
            self.results['external_blade_contact'] = {'detected': bool(contacts),
                                                      'collision_pairs': sorted(contacts)}
            self.save()
        finally:
            self.call(self.delete, DeleteEntity.Request(name=name), 15)
            self.destroy_subscription(subscription)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--check', choices=('basket', 'gripper', 'sphere', 'parking', 'rail', 'external_contact'), required=True)
    parser.add_argument('--camera', choices=('right', 'left', 'hand'), default='right')
    parser.add_argument('--cycles', type=int, default=5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = GeometryChecks(args.output)
    try:
        node.wait_for(lambda: 'gripper_joint' in node.q, 'joint states', 45)
        if args.check == 'basket':
            node.basket()
        elif args.check == 'gripper':
            node.gripper(args.cycles)
        elif args.check == 'sphere':
            node.sphere(args.camera)
        elif args.check == 'parking':
            node.parking()
        elif args.check == 'rail':
            node.rail()
        else:
            node.external_contact()
        print(json.dumps(node.results, indent=2), flush=True)
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
