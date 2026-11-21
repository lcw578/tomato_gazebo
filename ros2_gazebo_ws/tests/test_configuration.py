"""Regression checks for Gazebo dependency and world selection."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import json

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'scripts'))


class ConfigurationTest(unittest.TestCase):
    def test_harvest_station_defaults_to_zero_lift(self):
        config = json.loads((WORKSPACE / 'src/tomato_sim_gazebo/config/'
                             'simulation_baseline.json').read_text())
        self.assertEqual(config['candidate_harvest_station']['lift_m'], 0.0)

    def test_missing_external_weight_is_rejected_before_build(self):
        with tempfile.TemporaryDirectory() as directory:
            missing_weight = Path(directory) / 'missing.pt'
            result = subprocess.run(
                [sys.executable, str(WORKSPACE / 'scripts/prepare_overlay.py'), '--check'],
                env={**os.environ, 'TOMATO_YOLO_WEIGHTS': str(missing_weight)},
                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(str(missing_weight), result.stderr)

    def test_default_weights_live_in_the_standalone_project(self):
        with tempfile.TemporaryDirectory() as directory:
            env = {**os.environ, 'TOMATO_SIM_WORKSPACE': str(WORKSPACE)}
            env.pop('TOMATO_YOLO_WEIGHTS', None)
            result = subprocess.run(
                [sys.executable, str(WORKSPACE / 'scripts/simulation_config.py'), 'yolo_weights'],
                check=True, env=env, capture_output=True, text=True, cwd=directory)
        self.assertEqual(Path(result.stdout.strip()).parent, WORKSPACE.parent / 'assets/weights')
        self.assertTrue(Path(result.stdout.strip()).is_file())

    def test_world_name_is_read_from_sdf(self):
        from simulation_config import read_world

        with tempfile.TemporaryDirectory() as directory:
            world_path = Path(directory) / 'map_alias.world'
            world_path.write_text('<sdf version="1.6"><world name="actual_world"/></sdf>')
            self.assertEqual(read_world(world_path)[1].get('name'), 'actual_world')


if __name__ == '__main__':
    unittest.main()
