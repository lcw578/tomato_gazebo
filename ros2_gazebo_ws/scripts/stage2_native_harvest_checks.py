#!/usr/bin/env python3
"""Run unchanged native main control through consecutive physical harvest cycles."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
import yaml

from phase3_checks import Phase3Checks, WORKSPACE, pose_from_entity
from stage2_reset_payload import reset_payload
from tomato_sim_adapters.sim_gripper_node import CLOSED_RAD, FINGER_MULTIPLIERS
from tomato_sim_adapters.harvest_physics_arbiter import fruit_body_containment

CONTROL = WORKSPACE.parent / 'robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control'


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


@contextmanager
def process(command, log_path, hard_stop=False):
    """Stop only this runner's process group, including ROS console wrappers."""
    with log_path.open('w') as log:
        child = subprocess.Popen(command, cwd=WORKSPACE, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True,
                                 env={**os.environ, 'PYTHONUNBUFFERED': '1'})
        try:
            yield child
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL if hard_stop else signal.SIGINT)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=3)


class NativeHarvestChecks(Phase3Checks):
    def __init__(self, output, scene_file):
        self.rail_samples = []
        self.rail_samples_at_open = None
        self.grasp_seen = False
        super().__init__(output, scene_file)
        self.create_subscription(Bool, '/sim/harvest/gripper_closed', self.on_gripper, 10)

    def on_gripper(self, message):
        if message.data:
            self.grasp_seen = True
        elif self.grasp_seen:
            self.rail_samples_at_open = self.rail_samples.copy()

    def stop_rail(self):
        client = self.create_client(SetParameters, '/agv_always_go_straght_node/set_parameters')
        parameter = Parameter('is_continue_run', value=False).to_parameter_msg()
        response = self.call(client, SetParameters.Request(parameters=[parameter]), 'stop巡航')
        if not all(value.successful for value in response.results):
            raise RuntimeError('AGV adapter rejected the end-of-cycle parking request')

    def on_joints(self, message):
        super().on_joints(message)
        if 'machine_slide' in self.joints:
            self.rail_samples.append(self.joints['machine_slide'])

    def run_cycle(self, args, folder, scene):
        runtime = folder / 'runtime.json'
        runtime.unlink(missing_ok=True)
        result = {'mode': 'unchanged_native_main_control', 'passed': False,
                  'scene_file': str(args.scene_file), 'state_transitions': [],
                  'rail_start_position_m': self.joints.get('machine_slide')}
        arbiter = ['ros2', 'run', 'tomato_sim_adapters', 'harvest_physics_arbiter',
                   '--ros-args', '-p', 'use_sim_time:=true',
                   '-p', f'scene_file:={args.scene_file}', '-p', f'evidence_file:={runtime}']
        command = [str(WORKSPACE / '.venv_ros/bin/python'), '-c',
                   'from xc_robot_main_control.xc_robot_main_control_node import main; main()',
                   '--ros-args', '--params-file', str(args.main_config),
                   '-p', f'hand_eye_main_path:={args.calibration_dir}/']
        self.rail_samples.clear()
        self.rail_samples_at_open = None
        self.grasp_seen = False
        feedback_start = len(self.feedback)
        started = time.monotonic()
        try:
            with process(arbiter, folder / 'arbiter.log') as physics:
                self.wait_for(lambda: read_json(runtime).get('state') == 'ATTACHED_TO_STEM',
                              'fresh harvest arbiter', timeout=30)
                with process(command, folder / 'native_main.log', hard_stop=True) as main:
                    deadline = time.monotonic() + args.timeout
                    previous = None
                    while rclpy.ok() and time.monotonic() < deadline:
                        rclpy.spin_once(self, timeout_sec=.04)
                        evidence = read_json(runtime)
                        state = evidence.get('state')
                        if state != previous:
                            previous = state
                            result['state_transitions'].append({
                                'state': state, 'wall_elapsed_s': time.monotonic() - started})
                        if state == 'HELD_BY_GRIPPER' and 'held' not in result:
                            result['held'] = evidence
                            result['finger_closed_residual_m'] = {
                                side: abs(self.joints[f'{side}_finger_joint_mimic']
                                          - multiplier * CLOSED_RAD)
                                for side, multiplier in zip(('left', 'right'), FINGER_MULTIPLIERS)}
                        if main.poll() is not None or physics.poll() is not None:
                            raise RuntimeError('Native main control or harvest arbiter exited early')
                        log = (folder / 'native_main.log').read_text(errors='replace')
                        if ('held' in result and self.rail_samples_at_open
                                and '末端执行器打开成功' in log):
                            break
                        if re.search(r'错误码=3000[1356]|尝试4次均采摘失败', log):
                            raise RuntimeError('Native main control reported a motion or harvest failure')
                    else:
                        raise TimeoutError('Native harvest did not complete its payload-release open')
                self.stop_rail()
                self.wait_for(lambda: self.rail_samples_at_open,
                              'completed gripper-open observation', timeout=5)
                result['rail_drift_m'] = (max(self.rail_samples_at_open)
                                          - min(self.rail_samples_at_open))
                result['rail_drift_interval'] = 'native main startup through completed payload release open'
                evidence = self.wait_for(
                    lambda: read_json(runtime) if read_json(runtime).get('state') == 'RELEASED' else None,
                    'completed dynamic release', timeout=30)
                result['release'] = evidence['release']
                if result['state_transitions'][-1]['state'] != 'RELEASED':
                    result['state_transitions'].append({
                        'state': 'RELEASED', 'wall_elapsed_s': time.monotonic() - started})
                basket = self.transform('world', 'basket_link')
                rotation = Rotation.from_quat([basket.rotation.x, basket.rotation.y,
                                               basket.rotation.z, basket.rotation.w])
                origin = np.array([basket.translation.x, basket.translation.y, basket.translation.z])
                def check_containment(state):
                    position, attitude = pose_from_entity(state)
                    return fruit_body_containment(args.scene_file.parent / scene['release_sdf'],
                        (rotation.inv().apply(position - origin),
                         (rotation.inv() * attitude).as_quat()))

                state, settling = self.settled_payload(
                    timeout=120, model_name=result['release']['model_name'],
                    containment_check=check_containment)
                containment = check_containment(state)
                result.update(basket_containment=containment, settling=settling,
                              final_runtime=read_json(runtime),
                              rail_drift_through_settling_m=max(self.rail_samples)-min(self.rail_samples))
            log = (folder / 'native_main.log').read_text(errors='replace')
            result['original_skeleton_point_counts'] = [int(value) for value in re.findall(
                r'提取到的骨骼点数量:\s*(\d+)', log)]
            contact = result['held'].get('latest_cut_contact') or {}
            result['tcp_cutpoint_error_m'] = contact.get('tcp_cutpoint_error_m')
            result['crossing_angle_deg'] = contact.get('crossing_angle_deg')
            result['native_return_to_box_succeeded'] = '返回果箱成功' in log
            result['native_gripper_open_succeeded'] = '末端执行器打开成功' in log
            result['test_result_2'] = any(value == 2 for _, value in self.feedback[feedback_start:])
            result['passed'] = bool(
                contact and result['tcp_cutpoint_error_m'] < .005
                and result['crossing_angle_deg'] <= 30
                and max(result['finger_closed_residual_m'].values()) < .0004
                and result['held']['main_stem_contacts'] == 0
                and result['final_runtime']['main_stem_contacts'] == 0
                and result['final_runtime'].get('hold_error') is None
                and any(event.get('event') == 'CUT_SUCCESS'
                        for event in result['held']['cut_attempts'])
                and result['rail_drift_m'] < .0005
                and result['rail_drift_through_settling_m'] < .0005
                and result['test_result_2'] and result['native_return_to_box_succeeded']
                and result['native_gripper_open_succeeded']
                and result['original_skeleton_point_counts']
                and min(result['original_skeleton_point_counts']) > 10
                and containment['inside'] and result['release']['dynamics_enabled'])
        except Exception as error:
            result['error'] = str(error)
            result['final_runtime'] = read_json(runtime)
            try:
                self.stop_rail()
            except Exception as parking_error:
                result['parking_error'] = str(parking_error)
        result['wall_duration_s'] = time.monotonic() - started
        (folder / 'native_harvest.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({key: value for key, value in result.items()
                          if key not in ('held', 'final_runtime')}, indent=2), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-file', type=Path, required=True)
    parser.add_argument('--motion-config', type=Path, required=True)
    parser.add_argument('--calibration-dir', type=Path, required=True)
    parser.add_argument('--main-config', type=Path,
                        default=WORKSPACE / 'src/tomato_sim_gazebo/config/stage2_native_main.yaml')
    parser.add_argument('--output', type=Path, default=WORKSPACE / 'validation/stage2/native')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--exit-basket', action='store_true')
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')
    for field in ('scene_file', 'motion_config', 'calibration_dir', 'main_config', 'output'):
        setattr(args, field, getattr(args, field).resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    sources = [CONTROL / name for name in ('xc_robot_main_control_node.py', 'stemGrowthAndPoints.py')]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    summary = {'requested_consecutive_runs': args.runs, 'runs': [],
               'original_source_sha256_before': hashes, 'passed': False}
    rclpy.init()
    node = NativeHarvestChecks(args.output, args.scene_file)
    try:
        parameters = yaml.safe_load(args.motion_config.read_text())['/**']['ros__parameters']
        names = ['pose_reference', 'request_gripper_length', 'compensate_native_tool_offset',
                 'pick_waypoints_rad', 'box_waypoints_rad']
        client = node.create_client(GetParameters, '/sim_aubo_motion_service/get_parameters')
        response = node.call(client, GetParameters.Request(names=names), 'motion configuration')
        fields = ['string_value', 'double_value', 'bool_value',
                  'double_array_value', 'double_array_value']
        actual = {name: getattr(value, field)
                  for name, value, field in zip(names, response.values, fields)}
        for name in names:
            if isinstance(parameters[name], list):
                if list(actual[name]) != parameters[name]:
                    raise RuntimeError(f'Loaded motion route differs from {args.motion_config}')
            elif actual[name] != parameters[name]:
                raise RuntimeError(f'Loaded motion parameter differs from configuration: {name}')
        if (actual['pose_reference'] != 'sdk_flange'
                or not actual['compensate_native_tool_offset']
                or actual['request_gripper_length'] != .136):
            raise RuntimeError('Native acceptance requires SDK mode and full 136 mm tool compensation')
        summary['motion_parameters'] = parameters
        if any(name in node.get_node_names() for name in (
                'xc_robot_main_control_node', 'harvest_physics_arbiter')):
            raise RuntimeError('Stop the previous main control and arbiter before running acceptance')
        scene = json.loads(args.scene_file.read_text())
        node.stop_rail()
        for index in range(args.runs):
            reset_payload(node, args.exit_basket or index > 0, args.scene_file, args.motion_config)
            node.set_home()
            node.command_gripper('open')
            folder = args.output / f'run_{index + 1:02d}'
            folder.mkdir(exist_ok=True)
            result = node.run_cycle(args, folder, scene)
            summary['runs'].append(result)
            if not result['passed']:
                break
        after = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
        summary['original_source_sha256_after'] = after
        summary['original_sources_unchanged'] = after == hashes
        summary['passed'] = (len(summary['runs']) == args.runs
                             and all(result['passed'] for result in summary['runs'])
                             and summary['original_sources_unchanged'])
    except Exception as error:
        summary['error'] = str(error)
        raise
    finally:
        (args.output / 'native_harvest_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
    if not summary['passed']:
        raise SystemExit('Native harvest acceptance failed. Inspect native_harvest_summary.json.')


if __name__ == '__main__':
    main()
