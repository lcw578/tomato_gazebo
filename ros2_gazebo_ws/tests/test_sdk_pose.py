"""Hardware pose requests refer to the SDK flange frame."""
import math
from pathlib import Path
import sys
import threading
import types
import unittest

import numpy as np
import PyKDL as kdl
from geometry_msgs.msg import Transform
from xc_interfaces.srv import MoveToPose

from tomato_sim_adapters.arm_kinematics import ArmKinematics, OEM_HOME
from tomato_sim_adapters import sim_aubo_motion_service as motion

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_phase4_calibration as calibration


class SdkPoseTest(unittest.TestCase):
    def request(self, **values):
        defaults = dict(x=.04860708165105754, y=.6072221649501361,
                        z=.45528140574764536, rx=math.pi / 2,
                        ry=math.radians(70), rz=math.pi, mid2target_angle=0.)
        defaults.update(values)
        return MoveToPose.Request(**defaults)

    def sdk_waypoints(self, request):
        builder = getattr(motion, 'sdk_waypoints', None)
        self.assertTrue(callable(builder), 'SDK flange waypoint construction is missing')
        return builder(request)

    def test_sdk_frame_matches_forward_kinematics_reference(self):
        kin = ArmKinematics()
        forward = getattr(kin, 'fk_sdk', None)
        self.assertTrue(callable(forward), 'SDK forward pose conversion is missing')
        for q in (OEM_HOME, np.zeros(6), np.array([.3, -.4, 1.2, .7, -.6, .9])):
            with self.subTest(q=q):
                sdk_pose = forward(q)
                urdf_pose = kin.fk(q)
                np.testing.assert_allclose(list(urdf_pose.p - sdk_pose.p), [0, 0, .0235], atol=1e-12)
                self.assertLess(kdl.diff(urdf_pose, sdk_pose).rot.Norm(), 1e-12)

    def test_sdk_inverse_recovers_original_joint_pose(self):
        kin = ArmKinematics()
        inverse = getattr(kin, 'ik_sdk', None)
        self.assertTrue(callable(inverse), 'SDK inverse pose conversion is missing')
        wrist_pose = kin.fk(OEM_HOME)
        sdk_xyz = np.array(list(wrist_pose.p)) - [0, 0, .0235]
        solved = inverse(sdk_xyz, wrist_pose.M, OEM_HOME, alternatives=True)
        np.testing.assert_allclose(solved, OEM_HOME, atol=2e-5)

    def test_native_camera_calibration_uses_sdk_parent_origin(self):
        convert = getattr(calibration, 'calibration_matrix', None)
        self.assertTrue(callable(convert), 'Camera calibration reference conversion is missing')
        transform = Transform()
        transform.translation.x = .3
        transform.translation.y = -.2
        transform.translation.z = .5
        quaternion = kdl.Rotation.RPY(.2, -.3, .7).GetQuaternion()
        (transform.rotation.x, transform.rotation.y,
         transform.rotation.z, transform.rotation.w) = quaternion
        urdf = convert(transform, 'tcp')
        sdk = convert(transform, 'sdk_flange')
        np.testing.assert_allclose(sdk[:3, :3], urdf[:3, :3], atol=1e-12)
        np.testing.assert_allclose(sdk[:3, 3], [.3, -.2, .4765], atol=1e-12)
        self.assertEqual(transform.translation.z, .5)

    def test_confirmed_initial_pose_has_camera_above_horizontal_blades(self):
        pose = ArmKinematics().fk_tcp(OEM_HOME)
        camera_from_tcp = pose.M * kdl.Vector(.106042 + .0049, -.001376 + .0044,
                                               .025985 + .085)
        tilt = math.acos(abs(pose.M.UnitX().z()))
        self.assertLess(tilt, math.radians(2))
        self.assertGreater(camera_from_tcp.z(), .11)

    def test_calibration_cli_keeps_ros_parameters_out_of_optional_output(self):
        parse = getattr(calibration, 'parse_arguments', None)
        self.assertTrue(callable(parse), 'ROS-aware calibration argument parsing is missing')
        args = parse(['prepare_phase4_calibration.py', '--pose-reference', 'sdk_flange',
                      '--ros-args', '-p', 'use_sim_time:=true'])
        self.assertEqual(args.pose_reference, 'sdk_flange')
        self.assertEqual(args.output, calibration.workspace() / 'validation/phase4/calibration')

    def test_native_flange_positions_keep_hardware_approach_offset(self):
        request = self.request()
        midpoint, target, rotation = self.sdk_waypoints(request)
        np.testing.assert_allclose(target, [request.x, request.y, request.z])
        np.testing.assert_allclose(midpoint, [request.x, request.y - .1, request.z])
        self.assertLess(kdl.diff(kdl.Frame(rotation), kdl.Frame(
            kdl.Rotation.RPY(request.rx, request.ry, request.rz))).rot.Norm(), 1e-12)

    def test_native_angle_normalization_matches_both_hardware_sides(self):
        for y, angle, expected in ((.6, -90, -90), (.6, 100, -80),
                                   (.6, -110, 70), (-.6, 90, 90),
                                   (-.6, 110, -70), (-.6, -100, 80)):
            with self.subTest(y=y, angle=angle):
                request = self.request(y=y, mid2target_angle=math.radians(angle))
                midpoint, target, rotation = self.sdk_waypoints(request)
                sign = 1 if y > 0 else -1
                if expected == 90:
                    expected_offset = [0, 0 if sign > 0 else .1, .1]
                else:
                    expected_offset = [sign * .1 * math.sin(math.radians(expected)),
                                       -sign * .1 * math.cos(math.radians(expected)), 0]
                np.testing.assert_allclose(midpoint - target, expected_offset, atol=1e-12)
                expected_rotation = kdl.Rotation.RPY(request.rx, request.ry,
                                                      request.rz + math.radians(expected))
                if abs(expected) == 90:
                    expected_rotation = kdl.Rotation.RPY(math.radians(150), 0,
                                                        math.pi if y > 0 else 0)
                self.assertLess(kdl.diff(kdl.Frame(rotation), kdl.Frame(expected_rotation)).rot.Norm(), 1e-12)

    def test_nonfinite_native_pose_is_rejected(self):
        for field in ('x', 'y', 'z', 'rx', 'ry', 'rz', 'mid2target_angle'):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.sdk_waypoints(self.request(**{field: float('nan')}))

    def test_normal_move_service_uses_sdk_solver_for_native_request(self):
        kin = ArmKinematics()
        self.assertTrue(callable(getattr(kin, 'ik_sdk', None)), 'SDK solver is missing')
        executed = []
        published = types.SimpleNamespace(publish=lambda message: None)
        arm = types.SimpleNamespace(
            kin=kin, pose_reference='sdk_flange', gripper_length=.1,
            current=lambda: OEM_HOME.copy(), execute=lambda points, **kwargs: executed.append(points),
            midpoint=published, approach=published, state_lock=threading.Condition(),
            context=types.SimpleNamespace(ok=lambda: False),
            get_logger=lambda: types.SimpleNamespace(warning=lambda text: None))
        request = self.request()
        response = motion.SimArm.move_to_pose(arm, request, MoveToPose.Response())
        self.assertEqual(response.result, 0)
        self.assertEqual(len(executed), 2)
        midpoint_pose = kin.fk_sdk(executed[0][-1])
        target_pose = kin.fk_sdk(executed[1][-1])
        np.testing.assert_allclose(list(midpoint_pose.p),
                                   [request.x, request.y - .1, request.z], atol=2e-4)
        np.testing.assert_allclose(list(target_pose.p),
                                   [request.x, request.y, request.z], atol=2e-4)


if __name__ == '__main__':
    unittest.main()
