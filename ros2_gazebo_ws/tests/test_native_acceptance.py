"""Regression checks for native harvest acceptance and launch overrides."""
import ast
import importlib.util
import math
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities import evaluate_parameters

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'scripts'))


def load_method(path, name, namespace):
    tree = ast.parse(path.read_text())
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class NativeAcceptanceTest(unittest.TestCase):
    def test_payload_respawn_waits_for_deferred_delete(self):
        from stage2_reset_payload import reset_payload
        import json
        import tempfile

        reads = 0
        spawned = False

        def call(client, request, label, timeout=60):
            nonlocal reads, spawned
            if client == '/get_model_list':
                reads += 1
                return types.SimpleNamespace(model_names=['fixture'] if reads < 3 else [])
            if client == '/spawn_entity':
                self.assertGreaterEqual(reads, 3, 'Spawn raced the deferred deletion')
                spawned = True

        def wait_for(predicate, label, timeout=30):
            for _ in range(10):
                if predicate():
                    return True
            self.fail(label)

        node = types.SimpleNamespace(create_client=lambda _, name: name, call=call, wait_for=wait_for)
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / 'fruit.sdf').write_text('<sdf><model name="fruit"/></sdf>')
            (folder / 'scene.json').write_text(json.dumps({
                'target_model': 'fixture', 'plant_include_pose': '0 0 0 0 0 0',
                'fruit_local_pose': '0 0 0 0 0 0', 'release_sdf': 'fruit.sdf'}))
            reset_payload(node, False, folder / 'scene.json')
        self.assertTrue(spawned)

    def test_import_does_not_read_an_unused_default_scene(self):
        path = WORKSPACE / 'scripts/phase3_checks.py'
        spec = importlib.util.spec_from_file_location('phase3_import_check', path)
        module = importlib.util.module_from_spec(spec)
        read_text = Path.read_text

        def reject_default_scene(file_path, *args, **kwargs):
            if file_path.name == 'phase3_scene.json':
                raise FileNotFoundError('The default generated scene is absent')
            return read_text(file_path, *args, **kwargs)

        with patch.object(Path, 'read_text', reject_default_scene):
            spec.loader.exec_module(module)

    def settle(self, angular_speed, containment=None, moving_pose=False):
        count = 0
        vector = lambda x: types.SimpleNamespace(x=x, y=0., z=0.)
        pose = types.SimpleNamespace(position=vector(0.),
                                     orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.))

        def entity_state(name):
            nonlocal count
            count += 1
            if moving_pose:
                if moving_pose == 'oscillating':
                    phase = math.sin(4 * math.pi * (count-1) * .1) if count < 10 else 0.
                    pose.position.x = .0009 * phase
                    angle = math.radians(.4) * phase
                else:
                    pose.position.x = min(count, 10) * .0005
                    angle = min(count, 10) * .03
                pose.orientation.z = math.sin(angle / 2)
                pose.orientation.w = math.cos(angle / 2)
            return types.SimpleNamespace(pose=pose, twist=types.SimpleNamespace(
                linear=vector(0.), angular=vector(angular_speed(count))))

        node = types.SimpleNamespace(entity_state=entity_state,
            scene={'target_model': 'fixture'},
            get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(
                nanoseconds=int(count * .1 * 1e9))))
        namespace = {'time': time, 'math': math,
                     'rclpy': types.SimpleNamespace(spin_once=lambda *args, **kwargs: None)}
        method = load_method(WORKSPACE / 'scripts/phase3_checks.py', 'settled_payload', namespace)
        options = {'containment_check': containment} if containment else {}
        return method(node, model_name='fixture', timeout=1, **options)[1]

    def test_rotating_payload_must_stop_before_stability_window(self):
        result = self.settle(lambda sample: 3. if sample < 10 else 0.)
        self.assertGreaterEqual(result['sample_sim_time_s'], 1.5)
        self.assertEqual(result['angular_speed_rad_s'], 0.)
        self.assertTrue(all(sample['angular_speed_rad_s'] < .1
                            for sample in result['samples']))

    def test_pose_motion_restarts_the_stability_window(self):
        result = self.settle(lambda sample: .3 if sample < 10 else 0., moving_pose=True)
        self.assertGreaterEqual(result['sample_sim_time_s'], 1.5)
        self.assertLessEqual(result['maximum_position_change_m'], .001)
        self.assertLessEqual(result['maximum_rotation_change_deg'], .5)

    def test_stability_bounds_cover_both_extremes_of_an_oscillation(self):
        result = self.settle(lambda sample: .09, moving_pose='oscillating')
        positions = [sample['position_world_m'][0] for sample in result['samples']]
        self.assertLessEqual(max(positions) - min(positions), .001)
        angles = [math.degrees(2 * math.asin(sample['rotation_world_xyzw'][2]))
                  for sample in result['samples']]
        self.assertLessEqual(max(angles) - min(angles), .5)

    def test_containment_must_hold_throughout_stability_window(self):
        calls = 0

        def containment(state):
            nonlocal calls
            calls += 1
            return {'inside': calls >= 10, 'sphere_count': 12}

        result = self.settle(lambda sample: 0., containment)
        self.assertGreaterEqual(result['sample_sim_time_s'], 1.5)
        self.assertTrue(all(sample['containment']['inside'] for sample in result['samples']))

    def test_explicit_launch_switch_overrides_native_configuration(self):
        namespace = {'LaunchConfiguration': LaunchConfiguration, 'ParameterValue': ParameterValue,
                     'Node': Node, 'use_time': {'use_sim_time': True}}
        method = load_method(WORKSPACE / 'src/tomato_sim_gazebo/launch/phase2.launch.py',
                             'motion_service', namespace)
        context = LaunchContext()
        context.launch_configurations.update(
            motion_config=str(WORKSPACE / 'src/tomato_sim_gazebo/config/stage2_native_motion.yaml'),
            action_timeout_scale='', request_gripper_length='', pose_reference='',
            compensate_native_tool_offset='false')
        node = method(context)[0]
        parameters = evaluate_parameters(context, node._Node__parameters)
        config_index = next(index for index, value in enumerate(parameters) if isinstance(value, Path))
        switch_index = max(index for index, value in enumerate(parameters)
                           if isinstance(value, dict) and 'compensate_native_tool_offset' in value)
        self.assertGreater(switch_index, config_index)
        self.assertIs(parameters[switch_index]['compensate_native_tool_offset'], False)


if __name__ == '__main__':
    unittest.main()
