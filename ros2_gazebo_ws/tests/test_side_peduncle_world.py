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
            self.assertAlmostEqual(np.linalg.norm(tip - anchor), .11, places=9)
            self.assertGreater(variant['stalk_centerline_length_m'], .11)
            self.assertEqual(variant['stalk_curve_control_point_count'], 4)
            self.assertAlmostEqual(variant['stalk_base_diameter_m'], .008)
            self.assertAlmostEqual(variant['stalk_tip_diameter_m'], .0064)
            np.testing.assert_allclose(variant['anchor_calibration_offset_world_m'],
                                       [.00335, .00153, .00033], atol=1e-12)
            cut = np.array(scene['cut_point_world'])
            seam = np.array(scene['physical_cut_seam_world_m'])
            self.assertLess(np.linalg.norm(cut-seam), .0036)
            self.assertAlmostEqual(np.linalg.norm(cut-seam), .0025, places=5)
            self.assertGreater(np.linalg.norm(seam-anchor), .045)
            self.assertGreater(np.linalg.norm(tip-cut), .045)
            self.assertLess(variant['cut_seam_gap_m'], 1e-10)
            self.assertFalse(scene['v4_integrity']['all_original_entities_present'])
            model = ET.parse(Path(directory) / scene['release_sdf']).find('model/link')
            stem_collisions = [c for c in model.findall('collision')
                               if c.get('name').startswith('contact_stalk_')]
            self.assertEqual(len(stem_collisions), 2)
            self.assertTrue(all(c.find('surface/contact/collide_without_contact').text == 'true'
                                for c in stem_collisions))
            self.assertEqual(len([c for c in model.findall('collision')
                                  if c.find('geometry/sphere') is not None]), 12)
            self.assertTrue(any(v.find('geometry/mesh') is not None
                                for v in model.findall('visual')
                                if v.get('name') == 'contact_stalk_visual_fixture'))

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
            self.assertAlmostEqual(variant['stalk_chord_length_m'], .11)
            self.assertLess(variant['cut_seam_gap_m'], 1e-7)

    def test_side_stem_preserves_anchor_and_non_target_map_poses(self):
        for direction, sign in (('negative-x', -1), ('positive-x', 1)):
            with self.subTest(direction=direction), tempfile.TemporaryDirectory(
                    dir=WORKSPACE / 'validation') as directory:
                scene_path = side_world.prepare(Path(directory), direction, -20.)
                scene = json.loads(scene_path.read_text())
                tangent = np.array(scene['peduncle_tangent_world'])
                np.testing.assert_allclose(tangent, [sign*np.cos(np.deg2rad(20)),
                                                     0, -np.sin(np.deg2rad(20))], atol=1e-10)
                original = ET.parse(scene['source_world']).getroot().find('world')
                derived = ET.parse(scene['world']).getroot().find('world')
                poses = {node.findtext('name'): node.findtext('pose')
                         for node in derived.findall('include')}
                for node in original.findall('include'):
                    name = node.findtext('name')
                    if name not in (scene['plant_model'], scene['target_template']):
                        self.assertEqual(poses[name], node.findtext('pose'))
                model = ET.parse(Path(directory) / scene['release_sdf']).find('model/link')
                fruit_spheres = [node for node in model.findall('collision')
                                 if not node.get('name').startswith('contact_stalk_')
                                 and node.find('geometry/sphere') is not None]
                self.assertEqual(len(fruit_spheres), 12)
                self.assertLess(scene['acceptance_variant']['cut_seam_gap_m'], 1e-7)

    def test_lowered_fixture_tracks_zero_lift_robot_station(self):
        with tempfile.TemporaryDirectory(dir=WORKSPACE / 'validation') as directory:
            scene_path = side_world.prepare(Path(directory), 'positive-x', -20.,
                                            single_plant=True, target_lowering_m=.85)
            scene = json.loads(scene_path.read_text())
            variant = scene['acceptance_variant']
            anchor = np.asarray(variant['anchor_world_m'])
            original_anchor = np.asarray(variant['original_anchor_world_m'])
            np.testing.assert_allclose(anchor[2], original_anchor[2] - .85, atol=1e-9)
            self.assertAlmostEqual(variant['recommended_slide_m'],
                                   1.25 + anchor[0] - original_anchor[0], places=9)

    def test_full_v4_replaces_only_target_stalk_with_curved_mesh(self):
        with tempfile.TemporaryDirectory(dir=WORKSPACE / 'validation') as directory:
            scene_path = side_world.prepare(Path(directory), 'positive-x', -20.)
            scene = json.loads(scene_path.read_text())
            world = ET.parse(scene['world']).find('world')
            includes = world.findall('include')
            self.assertGreaterEqual(sum(node.findtext('name').startswith('plant_')
                                         for node in includes), 50)
            self.assertGreaterEqual(sum(node.findtext('name').startswith('fruit_truss_')
                                         for node in includes), 1)
            self.assertFalse(scene['collision_changes']['fixture_removes_other_plants_and_trusses'])
            self.assertGreaterEqual(scene['v4_integrity']['original_entities_preserved'],
                                    len(ET.parse(scene['source_world']).findall('world/include')) - 1)
            model = ET.parse(Path(directory) / scene['release_sdf']).find('model/link')
            self.assertTrue(any(node.get('name') == 'contact_stalk_visual_fixture'
                                for node in model.findall('visual')))
            self.assertEqual(len([node for node in model.findall('collision')
                                  if node.get('name', '').startswith(
                                      'contact_stalk_fixture_collision_')]), 2)


if __name__ == '__main__':
    unittest.main()
