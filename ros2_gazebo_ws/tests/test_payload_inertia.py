"""Keep released fruit mass centered on its physical sphere geometry."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'scripts'))

import prepare_v4_harvest_world as harvest_world


class PayloadInertiaTest(unittest.TestCase):
    def test_generated_target_uses_twelve_fruit_spheres_for_mass_distribution(self):
        original_output = harvest_world.common.OUT
        self.addCleanup(setattr, harvest_world.common, 'OUT', original_output)
        with tempfile.TemporaryDirectory(dir=WORKSPACE / 'validation') as directory:
            output = Path(directory)
            harvest_world.prepare(output=output)
            link = ET.parse(output / 'models/phase3_fruit_truss_9_1/model.sdf').find('model/link')
        inertial = link.find('inertial')
        self.assertEqual(float(inertial.findtext('mass')), 0.15)
        pose = [float(value) for value in inertial.findtext('pose', '0 0 0 0 0 0').split()]
        expected_pose = [0.008682392304090835, -0.004107233774080917,
                         -0.15338566754103336, 0.0, 0.0, 0.0]
        for actual, expected in zip(pose, expected_pose):
            self.assertAlmostEqual(actual, expected, places=12)
        expected_inertia = {
            'ixx': 0.0009791524326472104,
            'iyy': 0.0009949788050150534,
            'izz': 0.0000953375123356528,
            'ixy': -0.0000137215945203633,
            'ixz': -0.0000113597515904585,
            'iyz': 0.0000310468352895833,
        }
        for name, expected in expected_inertia.items():
            self.assertAlmostEqual(float(inertial.findtext(f'inertia/{name}')),
                                   expected, places=14)

    def test_tensor_includes_sphere_inertia_and_parallel_axis_terms(self):
        link = ET.fromstring('''<link name="fruit">
          <pose>5 6 7 0.1 0.2 0.3</pose>
          <inertial><pose>4 5 6 1 2 3</pose><mass>2</mass></inertial>
          <collision name="fruit_a"><pose>-1 -2 -3 0 0 0</pose>
            <geometry><sphere><radius>0.1</radius></sphere></geometry></collision>
          <collision name="fruit_b"><pose>1 2 3 0 0 0</pose>
            <geometry><sphere><radius>0.1</radius></sphere></geometry></collision>
          <collision name="contact_stalk_test"><pose>99 99 99 0 0 0</pose>
            <geometry><sphere><radius>1</radius></sphere></geometry></collision>
        </link>''')
        harvest_world.set_fruit_sphere_inertia(link)
        self.assertEqual(float(link.findtext('inertial/mass')), 2.0)
        self.assertEqual([float(value) for value in link.findtext('inertial/pose').split()],
                         [0.0] * 6)
        expected = {'ixx': 26.008, 'iyy': 20.008, 'izz': 10.008,
                    'ixy': -4.0, 'ixz': -6.0, 'iyz': -12.0}
        for name, value in expected.items():
            self.assertAlmostEqual(float(link.findtext(f'inertial/inertia/{name}')), value)


if __name__ == '__main__':
    unittest.main()
