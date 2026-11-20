"""Configured basket routes use their complete ordered joint trajectory."""
import types
import unittest

import numpy as np

from tomato_sim_adapters.arm_kinematics import OEM_HOME
from tomato_sim_adapters import sim_aubo_motion_service as motion
from tomato_sim_adapters.sim_aubo_motion_service import SimArm


class BoxWaypointsTest(unittest.TestCase):
    def test_flat_parameter_is_validated_as_six_joint_waypoints(self):
        parse = getattr(motion, 'parse_box_waypoints', None)
        self.assertTrue(callable(parse), 'Basket route parameter validation is missing')
        np.testing.assert_array_equal(parse([0.1] * 6 + [0.2] * 6),
                                      [[0.1] * 6, [0.2] * 6])
        self.assertEqual(parse(None).shape, (0, 6))
        self.assertEqual(parse([]).shape, (0, 6))
        for invalid in ([0.1] * 5, [[0.1] * 6], [float('nan')] * 6,
                        [float('inf')] * 6, [4.0] * 6):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                parse(invalid)

    def arm(self, points):
        executed = []
        arm = types.SimpleNamespace(
            box_waypoints=points,
            box=np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
            midpoint_q=np.zeros(6),
            approach_q=[],
            end_approach=lambda: None,
            execute=lambda path, **kwargs: executed.append(np.asarray(path).copy()),
        )
        arm.retreat = types.MethodType(SimArm.retreat, arm)
        return arm, executed

    def test_configured_route_keeps_prehome_waypoint_and_order(self):
        route = np.array([[0.7, -0.6, 1.1, -1.5, -1.5, -2.8],
                          OEM_HOME,
                          [0.8, -1.7, 1.4, 0.7, 1.0, 2.7]])
        arm, executed = self.arm(route)
        response = SimArm.return_box(arm, None, types.SimpleNamespace(result=0))
        self.assertEqual(response.result, 0)
        self.assertEqual(len(executed), 1)
        np.testing.assert_array_equal(executed[0], route)
        self.assertIsNone(arm.midpoint_q)

    def test_configured_route_runs_after_saved_cartesian_retreat(self):
        route = np.array([[0.7, -0.6, 1.1, -1.5, -1.5, -2.8]])
        arm, executed = self.arm(route)
        arm.approach_q = [np.full(6, 0.1), np.full(6, 0.2), np.full(6, 0.3)]
        response = SimArm.return_box(arm, None, types.SimpleNamespace(result=0))
        self.assertEqual(response.result, 0)
        self.assertEqual(len(executed), 2)
        np.testing.assert_array_equal(executed[0], [np.full(6, 0.2),
                                                  np.full(6, 0.1), np.zeros(6)])
        np.testing.assert_array_equal(executed[1], route)
        self.assertEqual(arm.approach_q, [])

    def test_empty_route_uses_legacy_home_and_box_target(self):
        arm, executed = self.arm([])
        response = SimArm.return_box(arm, None, types.SimpleNamespace(result=0))
        self.assertEqual(response.result, 0)
        np.testing.assert_array_equal(executed[0], [OEM_HOME, arm.box])


if __name__ == '__main__':
    unittest.main()
