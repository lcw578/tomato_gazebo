"""Regression checks for the confirmed arm mounting and basket release pose."""
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as kdl

from tomato_sim_adapters.arm_kinematics import ArmKinematics, OEM_HOME


class BasketTargetTest(unittest.TestCase):
    def test_basket_is_mounted_on_the_lift_platform(self):
        description = Path(__file__).resolve().parents[1] / (
            'src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro')
        root = ET.parse(description).getroot()
        mount = next(joint for joint in root.findall('joint')
                     if joint.get('name') == 'basket_mount')
        self.assertEqual(mount.find('parent').get('link'), 'lift_platform')

    def test_release_target_tracks_basket_at_both_lift_heights(self):
        kin = ArmKinematics()
        for lift in (0.0, 0.5):
            with self.subTest(lift=lift):
                basket = kdl.Frame(kdl.Vector(-0.533, 0.01, -0.88 - lift))
                arm = kdl.Frame(kdl.Rotation.RPY(np.pi, 0, np.pi),
                                kdl.Vector(0.002, 0.004, -0.704 - lift))
                base_from_basket = arm.Inverse() * basket
                target = kin.ik_basket(base_from_basket, OEM_HOME)
                self.assertLess(np.max(np.abs(target - OEM_HOME)), np.pi)
                actual = base_from_basket.Inverse() * kin.fk_tcp(target)
                expected_z = min(-0.22, (basket.Inverse() * arm).p.z() - 0.20)
                self.assertLess((actual.p - kdl.Vector(0, 0, expected_z)).Norm(), 0.0002)
                self.assertLess(actual.p.z(), (basket.Inverse() * arm).p.z() - 0.199)


if __name__ == '__main__':
    unittest.main()
