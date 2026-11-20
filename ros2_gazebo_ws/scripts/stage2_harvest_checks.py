#!/usr/bin/env python3
"""Measure a truth-driven harvest through the normal simulation services."""
import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from xc_interfaces.srv import MoveToPose

from phase3_checks import Phase3Checks, WORKSPACE, pose_from_entity
from tomato_sim_adapters.harvest_physics_arbiter import fruit_body_containment
from tomato_sim_adapters.sim_gripper_node import CLOSED_RAD, FINGER_MULTIPLIERS


class TruthHarvestChecks(Phase3Checks):
    def __init__(self, output, runtime, roll_degrees):
        super().__init__(output)
        self.runtime_path = Path(runtime)
        self.roll = Rotation.from_euler('x', roll_degrees, degrees=True)

    def target_pose(self, point_world, tangent_world):
        point, rotation = super().target_pose(point_world, tangent_world)
        return point, rotation * self.roll

    def runtime(self):
        return json.loads(self.runtime_path.read_text())

    def run_truth_harvest(self):
        self.wait_for(lambda: 'machine_slide' in self.joints, 'joint states', timeout=90)
        self.command_gripper('open')
        self.set_home()
        before = self.runtime()
        feedback_start = len(self.feedback)
        self.move_to(self.target, self.tangent, expect_feedback=2)
        self.wait_for(lambda: self.x1_values and self.x1_values[-1][1] == 1,
                      'gripper input X1 at the cut point', timeout=5)
        x1_at_cut = self.x1_values[-1][1]
        result_2 = any(value == 2 for _, value in self.feedback[feedback_start:])
        tcp_grasp, rotation_grasp = self.world_tcp_transform()
        fruit_grasp, _ = pose_from_entity(self.entity_state(self.scene['target_model']))
        self.command_gripper('close')
        self.wait_for(lambda: self.runtime().get('state') == 'HELD_BY_GRIPPER',
                      'contact-authorized cut and grasp', timeout=15)
        held = self.runtime()
        residuals = {}
        for side, multiplier in zip(('left', 'right'), FINGER_MULTIPLIERS):
            key = f'{side}_finger_joint_mimic'
            residuals[side] = abs(self.joints[key] - multiplier * CLOSED_RAD)
        self.call(self.retreat, MoveToPose.Request(), 'retreat holding fruit', timeout=300)
        tcp_retracted, rotation_retracted = self.world_tcp_transform(
            after_stamp=self.get_clock().now())
        expected = tcp_retracted + rotation_retracted.apply(
            rotation_grasp.inv().apply(fruit_grasp - tcp_grasp))
        follow_samples = []

        def payload_reached_retreat():
            now = self.get_clock().now().nanoseconds / 1e9
            if follow_samples and now - follow_samples[-1]['sim_time_s'] < 0.05:
                return None
            position, _ = pose_from_entity(self.entity_state(self.scene['target_model']))
            error = float(np.linalg.norm(position - expected))
            follow_samples.append({'sim_time_s': self.get_clock().now().nanoseconds / 1e9,
                                   'position_world_m': position.tolist(), 'error_m': error})
            return position if error < 0.005 else None

        fruit_retracted = self.wait_for(payload_reached_retreat,
                                        'payload follows the stopped TCP', timeout=5)
        self.return_to_box_pose()
        before_release = self.entity_state(self.scene['target_model'])
        self.command_gripper('open')
        self.wait_for(lambda: self.runtime().get('state') == 'RELEASED',
                      'dynamic payload release', timeout=30)
        release = self.runtime()['release']
        state, settling = self.settled_payload(timeout=180, model_name=release['model_name'])
        world_from_basket = self.transform('world', 'basket_link')
        t, q = world_from_basket.translation, world_from_basket.rotation
        basket_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        basket_position = np.array([t.x, t.y, t.z])
        fruit_final, fruit_rotation = pose_from_entity(state)
        local = basket_rotation.inv().apply(fruit_final - basket_position)
        local_rotation = (basket_rotation.inv() * fruit_rotation).as_quat()
        containment = fruit_body_containment(
            WORKSPACE / 'validation/phase3' / self.scene.get(
                'release_sdf', 'models/phase3_fruit_truss_9_1/model.sdf'),
            (local, local_rotation))
        release_position, _ = pose_from_entity(before_release)
        result = {
            'mode': 'ground_truth_service_drive',
            'target_model': self.scene['target_model'],
            'rail_station_m': self.joints['machine_slide'],
            'tcp_cutpoint_error_m': float(np.linalg.norm(tcp_grasp - self.target)),
            'input_x1_at_cut': x1_at_cut,
            'test_result_2': result_2,
            'finger_closed_residual_m': residuals,
            'blade_contact': held.get('latest_cut_contact'),
            'blade_sample': held.get('latest_blade_sample'),
            'cut_success': any(event.get('event') == 'CUT_SUCCESS'
                               for event in held['cut_attempts'][len(before['cut_attempts']):]),
            'main_stem_contacts': held['main_stem_contacts'] - before['main_stem_contacts'],
            'held_follow_error_m': float(np.linalg.norm(fruit_retracted - expected)),
            'held_follow_endpoint_samples': follow_samples,
            'held_follow_endpoint_convergence_s': (
                follow_samples[-1]['sim_time_s'] - follow_samples[0]['sim_time_s']),
            'tcp_grasp_world_m': tcp_grasp.tolist(),
            'tcp_retracted_world_m': tcp_retracted.tolist(),
            'fruit_grasp_world_m': fruit_grasp.tolist(),
            'held_displacement_m': float(np.linalg.norm(fruit_retracted - fruit_grasp)),
            'release_position_world_m': release_position.tolist(),
            'settled_root_basket_m': local.tolist(),
            'settled_rotation_basket_xyzw': local_rotation.tolist(),
            'basket_containment': containment,
            'settling': settling,
            'release': self.runtime().get('release'),
        }
        result['passed'] = bool(
            result['tcp_cutpoint_error_m'] < 0.005
            and result['input_x1_at_cut'] == 1 and result['test_result_2']
            and max(residuals.values()) < 0.0004
            and result['cut_success'] and result['main_stem_contacts'] == 0
            and result['held_follow_error_m'] < 0.005
            and containment['inside'])
        path = self.output / 'truth_harvest.json'
        path.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        if not result['passed']:
            raise RuntimeError('Truth-driven harvest acceptance failed, see truth_harvest.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('validation/stage2'))
    parser.add_argument('--runtime', type=Path, default=Path('validation/stage2/runtime.json'))
    parser.add_argument('--roll-degrees', type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = TruthHarvestChecks(args.output, args.runtime, args.roll_degrees)
    try:
        node.run_truth_harvest()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
