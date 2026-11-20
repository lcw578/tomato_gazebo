"""Keep side-facing acceptance stems connected to the original plant."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'scripts'))
import prepare_side_peduncle_world as side_world


class SidePeduncleWorldTest(unittest.TestCase):
    def test_single_plant_fixture_has_a_geometric_midpoint_and_physical_payload(self):
        with tempfile.TemporaryDirectory(dir=WORKSPACE / 'validation') as directory:
            path = side_world.prepare(Path(directory), 'positive-x', -20., single_plant=True)
            scene = json.loads(path.read_text())
            world = ET.parse(scene['world']).find('world')
            names = [node.findtext('name') for node in world.findall('include')]
            self.assertEqual([name for name in names if name.startswith('plant_')],
                             [scene['plant_model']])
            self.assertEqual([name for name in names if name.startswith('fruit_truss_')],
                             [scene['target_model']])
            variant = scene['acceptance_variant']
            anchor = np.array(variant['anchor_world_m'])
            tip = np.array(variant['stem_tip_world_m'])
            np.testing.assert_allclose(scene['cut_point_world'], (anchor + tip) / 2, atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(tip - anchor), .24)
            self.assertLess(variant['cut_seam_gap_m'], 1e-10)
            self.assertFalse(scene['v4_integrity']['all_original_entities_present'])
            model = ET.parse(Path(directory) / scene['release_sdf']).find('model/link')
            self.assertEqual(len([c for c in model.findall('collision')
                                  if c.find('geometry/sphere') is not None]), 12)

    def test_longer_oblique_stem_keeps_a_connected_cut_seam(self):
        with tempfile.TemporaryDirectory(dir=WORKSPACE / 'validation') as directory:
            path = side_world.prepare(Path(directory), 'negative-x', -20.,
                                      azimuth_offset_deg=-25., length_scale=2.)
            scene = json.loads(path.read_text())
            azimuth = np.deg2rad(155.)
            elevation = np.deg2rad(-20.)
            np.testing.assert_allclose(scene['peduncle_tangent_world'],
                                       [np.cos(elevation)*np.cos(azimuth),
                                        np.cos(elevation)*np.sin(azimuth),
                                        np.sin(elevation)], atol=1e-10)
            variant = scene['acceptance_variant']
            self.assertEqual(variant['stalk_length_scale'], 2.)
            self.assertLess(variant['anchor_displacement_m'], 1e-10)
            self.assertLess(variant['cut_seam_gap_m'], 1e-7)

    def test_side_stem_preserves_anchor_and_original_map_poses(self):
        for direction, sign in (('negative-x', -1), ('positive-x', 1)):
            with self.subTest(direction=direction), tempfile.TemporaryDirectory(
                    dir=WORKSPACE / 'validation') as directory:
                scene_path = side_world.prepare(Path(directory), direction, -20.)
                scene = json.loads(scene_path.read_text())
                tangent = np.array(scene['peduncle_tangent_world'])
                np.testing.assert_allclose(tangent, [sign*np.cos(np.deg2rad(20)),
                                                     0, -np.sin(np.deg2rad(20))], atol=1e-10)
                self.assertLess(scene['acceptance_variant']['anchor_displacement_m'], 1e-10)
                self.assertTrue(scene['v4_integrity']['all_original_include_poses_unchanged'])
                original = ET.parse(scene['source_world']).getroot().find('world')
                derived = ET.parse(scene['world']).getroot().find('world')
                poses = {node.findtext('name'): node.findtext('pose')
                         for node in derived.findall('include')}
                for node in original.findall('include'):
                    self.assertEqual(poses[node.findtext('name')], node.findtext('pose'))
                model = ET.parse(Path(directory) / scene['release_sdf']).find('model/link')
                fruit_spheres = [node for node in model.findall('collision')
                                 if not node.get('name').startswith('contact_stalk_')
                                 and node.find('geometry/sphere') is not None]
                self.assertEqual(len(fruit_spheres), 12)
                self.assertLess(scene['acceptance_variant']['cut_seam_gap_m'], 1e-7)


if __name__ == '__main__':
    unittest.main()
