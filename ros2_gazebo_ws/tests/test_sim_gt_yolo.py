"""Geometry and real ROS response-contract regressions for the GT detector."""
from array import array
from collections import deque
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/tomato_sim_adapters'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from tomato_sim_adapters.gt_geometry_projection import (
    Geometry, load_scene_geometry, pose_matrix, render_depths, visible_detections,
)
from tomato_sim_adapters.sim_gt_yolo_service import (
    depth_image_metres, fill_response, select_camera_info,
)
from sensor_msgs.msg import CameraInfo, Image
from xc_interfaces.srv import YoloDetection
from stage2_gt_vision_checks import (
    REAL_CONTROL, cutter_crossing_angle, original_request_builder, original_vision_methods,
    point_error_components,
)


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.camera = np.array([[10., 0, 10], [0, 10., 10], [0, 0, 1.]])

    def test_sphere_raster_uses_surface_z_and_forward_world_transform(self):
        sphere = Geometry('stem', 'sphere', pose_matrix('1 0 2 0 0 0'), radius=.5)
        optical_from_world = pose_matrix('-1 0 0 0 0 0')
        depths = render_depths([sphere], optical_from_world, self.camera, 21, 21)
        self.assertAlmostEqual(depths['stem'][10, 10], 1.5)
        self.assertTrue(np.isinf(depths['stem'][10, 14]))
        self.assertAlmostEqual(depths['stem'][10, 12], 1.619011763, places=7)

    def test_cylinder_intersections_include_side_and_caps(self):
        cap = Geometry('stem', 'cylinder', pose_matrix('0 0 2 0 0 0'),
                       radius=.5, length=1.)
        side = Geometry('trunk', 'cylinder', pose_matrix('0 0 2 1.5707963267948966 0 0'),
                        radius=.5, length=1.)
        depths = render_depths([cap, side], np.eye(4), self.camera, 21, 21)
        self.assertAlmostEqual(depths['stem'][10, 10], 1.5)
        self.assertAlmostEqual(depths['trunk'][10, 10], 1.5)
        self.assertAlmostEqual(depths['trunk'][12, 10], 1.5)
        self.assertTrue(np.isinf(depths['trunk'][14, 10]))

    def test_mesh_depth_is_perspective_correct_and_order_independent(self):
        mesh = Geometry('tomato', 'mesh', np.eye(4),
                        vertices=np.array([[-1., -1., 1.], [2., -2., 2.], [0., 2., 2.]]),
                        faces=np.array([[0, 1, 2]]))
        depths = render_depths([mesh], np.eye(4), self.camera, 21, 21)
        # Screen barycentrics at (10, 10) are (1/4, 1/4, 1/2).
        self.assertAlmostEqual(depths['tomato'][10, 10], 1.6)
        farther = Geometry('tomato', 'sphere', pose_matrix('0 0 4 0 0 0'), radius=.2)
        reversed_depths = render_depths([farther, mesh], np.eye(4), self.camera, 21, 21)
        np.testing.assert_array_equal(depths['tomato'], reversed_depths['tomato'])

    def test_near_plane_crossing_mesh_is_clipped_and_behind_geometry_is_empty(self):
        mesh = Geometry('tomato', 'mesh', np.eye(4),
                        vertices=np.array([[-.1, -.1, -.1], [.2, -.1, 1.], [0., .2, 1.]]),
                        faces=np.array([[0, 1, 2]]))
        behind = Geometry('stem', 'sphere', pose_matrix('0 0 -1 0 0 0'), radius=.1)
        depths = render_depths([mesh, behind], np.eye(4), self.camera, 21, 21)
        self.assertGreater(np.count_nonzero(np.isfinite(depths['tomato'])), 0)
        self.assertEqual(np.count_nonzero(np.isfinite(depths['stem'])), 0)
        self.assertTrue(np.all(depths['tomato'][np.isfinite(depths['tomato'])] > 0))

    def test_visibility_rejects_occlusion_background_and_invalid_observed_depth(self):
        projected = {'stem': np.array([[1., 1., 1., 1., 1., np.inf]])}
        observed = np.array([[1.005, .97, 1.03, 0., np.nan, 1.]])
        detections = visible_detections(projected, observed, .012)
        self.assertEqual(len(detections), 1)
        np.testing.assert_array_equal(detections[0].mask, [[255, 0, 0, 0, 0, 0]])
        self.assertEqual(detections[0].bbox, [0., 0., 1., 1.])
        self.assertEqual(detections[0].projected_pixels, 5)

    def test_split_stalk_geometries_union_and_known_foreground_occludes(self):
        near = Geometry('stem', 'sphere', pose_matrix('-.15 0 2 0 0 0'), radius=.3)
        far = Geometry('stem', 'sphere', pose_matrix('.15 0 2 0 0 0'), radius=.3)
        tomato = Geometry('tomato', 'sphere', pose_matrix('0 0 1.8 0 0 0'), radius=.105)
        depths = render_depths([near, far, tomato], np.eye(4), self.camera, 21, 21)
        observed = np.minimum(depths['stem'], depths['tomato'])
        detections = visible_detections(depths, observed, .012)
        self.assertEqual([d.class_name for d in detections], ['stem', 'tomato'])
        self.assertEqual(detections[0].mask[10, 10], 0)
        self.assertEqual(detections[1].mask[10, 10], 255)

    def test_scene_loader_combines_both_models_and_applies_link_mesh_scale(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('plant', 'fruit', 'assets'):
                (root / 'models' / name).mkdir(parents=True)
            (root / 'models/assets/fruit.obj').write_text(
                'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1 2/2 3/3\n')
            (root / 'world.sdf').write_text('''<sdf><world name="test">
                <include><name>plant_instance</name><uri>model://plant</uri>
                <pose>1 0 0 0 0 0</pose></include>
                <include><name>fruit_instance</name><uri>model://fruit</uri>
                <pose>2 0 0 0 0 0</pose></include></world></sdf>''')
            (root / 'models/plant/model.sdf').write_text('''<sdf><model name="plant">
                <link name="plant"><pose>0 1 0 0 0 0</pose>
                <visual name="contact_stalk_visual_plant_0"><geometry><cylinder>
                <radius>.003</radius><length>.02</length></cylinder></geometry></visual>
                <visual name="g_v1_vine_visual"><geometry><mesh>
                <uri>model://assets/fruit.obj</uri></mesh></geometry></visual>
                </link></model></sdf>''')
            (root / 'models/fruit/model.sdf').write_text('''<sdf><model name="fruit">
                <link name="fruit">
                <visual name="contact_stalk_joint_1"><geometry><sphere>
                <radius>.003</radius></sphere></geometry></visual>
                <visual name="g_tr1_mesh_visual"><geometry><mesh>
                <uri>model://assets/fruit.obj</uri><scale>2 3 4</scale>
                </mesh></geometry></visual></link></model></sdf>''')
            scene_path = root / 'scene.json'
            scene_path.write_text(json.dumps({'world': str(root / 'world.sdf'),
                'plant_model': 'plant_instance', 'target_model': 'fruit_instance'}))
            geometry = load_scene_geometry(scene_path)
        self.assertEqual([g.class_name for g in geometry], ['stem', 'trunk', 'stem', 'tomato'])
        np.testing.assert_allclose(geometry[0].world_from_local[:3, 3], [1., 1., 0.])
        np.testing.assert_allclose(geometry[3].vertices[1], [2., 0., 0.])
        np.testing.assert_allclose(geometry[3].vertices[2], [0., 3., 0.])


class ServiceContractTest(unittest.TestCase):
    def test_depth_units_endianness_padding_and_float_metres(self):
        depth = Image(height=1, width=2, encoding='16UC1', is_bigendian=1, step=6,
                      data=array('B', b'\x03\xe8\x07\xd0\xff\xff'))
        np.testing.assert_array_equal(depth_image_metres(depth, 1000), [[1., 2.]])
        float_depth = Image(height=1, width=2, encoding='32FC1', step=8,
                            data=array('B', np.array([1.25, 2.5], dtype='<f4').tobytes()))
        np.testing.assert_array_equal(depth_image_metres(float_depth, 4000), [[1.25, 2.5]])

    def test_response_keeps_images_and_every_mask_index_aligned_when_stem_is_empty(self):
        request = YoloDetection.Request()
        request.color_image = Image(height=2, width=3, encoding='rgb8', step=9,
                                    data=array('B', bytes(range(18))))
        request.color_image.header.frame_id = 'cam_right_color_optical_frame'
        request.color_image.header.stamp.sec = 2
        request.depth_image = Image(height=2, width=3, encoding='16UC1', step=6,
                                    data=array('B', bytes(range(12))))
        depths = {'stem': np.full((2, 3), np.inf),
                  'tomato': np.array([[1., 1., np.inf], [np.inf, np.inf, np.inf]]),
                  'trunk': np.array([[np.inf, np.inf, np.inf], [np.inf, np.inf, 1.]])}
        detections = visible_detections(depths, np.ones((2, 3)), .012)
        response = fill_response(request, YoloDetection.Response(), detections)
        self.assertEqual(response.color_image, request.color_image)
        self.assertEqual(response.depth_image, request.depth_image)
        self.assertEqual(response.class_names, ['tomato', 'trunk'])
        self.assertEqual(list(response.class_ids), [1, 2])
        self.assertEqual(list(response.bboxes), [0., 0., 2., 1., 2., 1., 3., 2.])
        self.assertEqual(len(response.confidences), 2)
        self.assertEqual(len(response.masks), 2)
        for index, mask in enumerate(response.masks):
            self.assertEqual(mask.class_id, response.class_ids[index])
            self.assertEqual(mask.class_names, response.class_names[index])
            self.assertEqual(mask.confidence, response.confidences[index])
            self.assertEqual(mask.mask.encoding, 'mono8')
            self.assertEqual((mask.mask.height, mask.mask.width, mask.mask.step), (2, 3, 3))
            self.assertEqual(mask.mask.header, request.color_image.header)
            self.assertEqual(set(mask.mask.data), {0, 255})

    def test_camera_info_uses_requested_frame_and_historical_exposure(self):
        image = Image(height=2, width=3)
        image.header.frame_id = '/cam_right_color_optical_frame'
        image.header.stamp.sec = 5
        history = deque()
        for frame, second in [('cam_left_color_optical_frame', 5),
                              ('cam_right_color_optical_frame', 4),
                              ('cam_right_color_optical_frame', 6)]:
            info = CameraInfo(height=2, width=3)
            info.header.frame_id = frame
            info.header.stamp.sec = second
            info.k = [10., 0., 1., 0., 10., 1., 0., 0., 1.]
            history.append(info)
        self.assertEqual(select_camera_info(history, image, 2.).header.stamp.sec, 4)
        with self.assertRaises(ValueError):
            select_camera_info(history, image, .5)


class VisionDiagnosticTest(unittest.TestCase):
    def test_predicted_cutter_crossing_angle_treats_the_tangent_as_unsigned(self):
        self.assertEqual(cutter_crossing_angle(np.eye(4), np.eye(3), [-2., 0., 0.]), 0.)
        self.assertEqual(cutter_crossing_angle(np.eye(4), np.eye(3), [0., 1., 0.]), 90.)

    def test_world_cut_error_uses_forward_pose_and_signed_axial_projection(self):
        world_from_optical = np.array([[0., -1., 0., 1.], [1., 0., 0., 2.],
                                       [0., 0., 1., 3.], [0., 0., 0., 1.]])
        result = point_error_components([.002, .003, .004], world_from_optical,
                                        [1., 2., 3.], [2., 0., 0.])
        np.testing.assert_allclose(result['point_world_m'], [.997, 2.002, 3.004])
        self.assertAlmostEqual(result['total_error_m'], .005385164807)
        self.assertAlmostEqual(result['axial_error_m'], -.003)
        self.assertAlmostEqual(result['normal_error_m'], .004472135955)

    def test_original_main_request_block_preserves_rpy_and_zero_gripper_offset(self):
        source_path = REAL_CONTROL / 'xc_robot_main_control_node.py'
        logger = SimpleNamespace(info=lambda *args, **kwargs: None)
        methods = original_vision_methods(source_path, logger)
        methods.first_see_harvestable_tomato = 1
        methods.is_use_base_angle = True
        methods.is_two_stage = False
        methods.gripper_length = 0.
        request, base_angle, approach_angle = original_request_builder(source_path)(
            methods, .01, .7, .41, [0., .7, .4, 1.], [.005, .7, .42, 1.],
            [.005, .7, .42, 1.], '朝左', 0.)
        np.testing.assert_allclose([request.x, request.y, request.z], [.01, .7, .41])
        np.testing.assert_allclose([request.rx, request.ry, request.rz],
                                  [np.pi / 2, -70 * np.pi / 180, np.pi])
        self.assertAlmostEqual(base_angle, 1.3258176636680326)
        self.assertEqual(approach_angle, 0.)


if __name__ == '__main__':
    unittest.main()
