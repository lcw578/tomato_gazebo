"""The simulated service must include actual finger tracking in completion."""
import unittest

from tomato_sim_adapters.sim_gripper_node import gripper_reached


class GripperCompletionTest(unittest.TestCase):
    def test_accepts_aligned_closed_fingers(self):
        self.assertTrue(gripper_reached(0.4, (-0.0083996, -0.008398), 0.4))

    def test_rejects_gear_only_arrival_with_blocked_fingers(self):
        self.assertFalse(gripper_reached(0.4, (-0.01033446, -0.01024986), 0.4))

    def test_rejects_missing_finger_feedback(self):
        self.assertFalse(gripper_reached(0.4, (None, None), 0.4))


if __name__ == '__main__':
    unittest.main()
