#!/usr/bin/env python3
"""Record an exposure-synchronized GT YOLO and original stem-algorithm check."""
import argparse
import ast
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import time
import types

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from xc_interfaces.srv import MoveToPose, YoloDetection

from tomato_sim_adapters.gt_geometry_projection import transform_matrix
from tomato_sim_adapters.ros_vision_compat import TomatoSimCvBridge
from tomato_sim_adapters.sim_gt_yolo_service import depth_image_metres

WORKSPACE = Path(__file__).resolve().parents[1]
REAL_CONTROL = WORKSPACE.parent / 'robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control'


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def point_error_components(point_optical, world_from_optical, truth_world, tangent_world):
    """Measure signed axial and unsigned normal errors against the scene truth."""
    point_world = (world_from_optical @ np.r_[point_optical, 1.])[:3]
    error = point_world - np.asarray(truth_world)
    tangent = np.asarray(tangent_world, dtype=float)
    tangent /= np.linalg.norm(tangent)
    axial = float(error @ tangent)
    normal = error - axial * tangent
    return {'point_optical_m': np.asarray(point_optical).tolist(),
            'point_world_m': point_world.tolist(), 'error_vector_world_m': error.tolist(),
            'total_error_m': float(np.linalg.norm(error)), 'axial_error_m': axial,
            'normal_error_m': float(np.linalg.norm(normal))}


def cutter_crossing_angle(world_from_base, base_from_cutter_rotation, tangent_world):
    """Predict the arbiter's unsigned tangent-to-cutter-X angle in degrees."""
    cutter_x_world = world_from_base[:3, :3] @ base_from_cutter_rotation[:, 0]
    tangent = np.asarray(tangent_world, dtype=float)
    cosine = abs(float(cutter_x_world @ tangent)) / np.linalg.norm(tangent)
    return math.degrees(math.acos(float(np.clip(cosine, 0., 1.))))


def original_vision_methods(source_path, logger):
    """Load the original controller's vision methods without constructing its node."""
    import cv2
    source = ast.parse(source_path.read_text())
    original = next(item for item in source.body
                    if isinstance(item, ast.ClassDef) and item.name == 'XCRobotMainControlNode')
    names = {'classify_detections', 'calculate_iou', 'calculate_iou_stems', 'calculate_gap',
             'can_intersect_after_shift', 'combine_boxes', 'get_valid_depth_value',
             'get_neighborhood_depth', 'get_mask_average_depth', 'create_mid2target_angle'}
    methods = [item for item in original.body
               if isinstance(item, ast.FunctionDef) and item.name in names]
    if {item.name for item in methods} != names:
        raise ValueError('Original main-controller vision methods are incomplete')
    isolated = ast.ClassDef(name=original.name, bases=[], keywords=[], body=methods,
                           decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[isolated], type_ignores=[]))
    namespace = {'np': np, 'cv2': cv2, 'math': math}
    exec(compile(module, str(source_path), 'exec'), namespace)
    methods_object = namespace[original.name]()
    methods_object.get_logger = lambda: logger
    methods_object.is_real_camera_data = True
    methods_object.depth_scale = .001
    return methods_object


def original_request_builder(source_path):
    """Compile the original request-construction statements as an isolated function."""
    source = ast.parse(source_path.read_text())
    original = next(item for item in source.body
                    if isinstance(item, ast.ClassDef) and item.name == 'XCRobotMainControlNode')
    callback = next(item for item in original.body
                    if isinstance(item, ast.FunctionDef) and item.name == 'detection_callback')
    statements = None
    for node in ast.walk(callback):
        for field_name, value in ast.iter_fields(node):
            if not isinstance(value, list):
                continue
            for index, item in enumerate(value):
                if (isinstance(item, ast.Assign) and isinstance(item.targets[0], ast.Tuple)
                        and [getattr(target, 'id', '') for target in item.targets[0].elts]
                        == ['rx', 'ry', 'rz']):
                    end = next(position for position in range(index, len(value))
                               if isinstance(value[position], ast.Assign)
                               and isinstance(value[position].targets[0], ast.Attribute)
                               and value[position].targets[0].attr == 'target_point_request_')
                    statements = value[index:end]
    if statements is None:
        raise ValueError('Original main-controller request-construction block was not found')
    result = ast.Return(value=ast.Tuple(elts=[ast.Name(id=name, ctx=ast.Load())
                        for name in ('request', 'tangent_angle_by_base', 'mid2target_angle')],
                        ctx=ast.Load()))
    loop = ast.For(target=ast.Name(id='diagnostic_pair', ctx=ast.Store()),
                   iter=ast.List(elts=[ast.Constant(value=0)], ctx=ast.Load()),
                   body=statements + [result], orelse=[])
    arguments = ['self', 'X', 'Y', 'Z', 'base_point_left', 'base_point_right',
                 'base_point_top', 'growth_direction', 'tangent_angle']
    function = ast.FunctionDef(name='build_original_request', args=ast.arguments(
        posonlyargs=[], args=[ast.arg(arg=name) for name in arguments],
        kwonlyargs=[], kw_defaults=[], defaults=[]), body=[loop], decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
    namespace = {'math': math, 'np': np, 'MoveToPose': MoveToPose}
    exec(compile(module, str(source_path), 'exec'), namespace)
    return namespace['build_original_request']


def original_stem_result(calculator, stem, tomato, stem_mask, tomato_mask, color):
    """Observe the real function's returned skeleton without replacing its logic."""
    captured = {}
    target_code = calculator.determine_growth_direction_and_picking_point.__code__

    def capture_return(frame, event, value):
        if event == 'return' and frame.f_code is target_code:
            captured['thin'] = frame.f_locals.get('thin')

    previous_profile = sys.getprofile()
    stdout = io.StringIO()
    try:
        sys.setprofile(capture_return)
        with contextlib.redirect_stdout(stdout):
            result = calculator.determine_growth_direction_and_picking_point(
                0, stem, tomato, stem_mask, tomato_mask, color,
                (stem[0] + stem[2]) / 2, (tomato[0] + tomato[2]) / 2,
                (stem[2] - stem[0]) / (stem[3] - stem[1]), color.shape[1], color.shape[0])
    finally:
        sys.setprofile(previous_profile)
    return result, captured.get('thin'), stdout.getvalue()


class VisionChecks(Node):
    def __init__(self, root):
        super().__init__('stage2_gt_vision_checks')
        self.groups = {}
        self.latest_group = None
        self.joints = {}
        self.buffer = Buffer(cache_time=Duration(seconds=30.))
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(YoloDetection, '/yolo_detection')
        self.subscriptions_images = [self.create_subscription(
            kind, topic, lambda message, stream=stream: self.on_image(stream, message),
            qos_profile_sensor_data) for stream, kind, topic in (
                ('color', Image, root + '/color/image_raw'),
                ('depth', Image, root + '/aligned_depth_to_color/image_raw'),
                ('info', CameraInfo, root + '/color/camera_info'))]
        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 qos_profile_sensor_data)

    def on_image(self, stream, message):
        stamp = stamp_ns(message.header.stamp)
        group = self.groups.setdefault(stamp, {})
        group[stream] = message
        if len(group) == 3:
            self.latest_group = group.copy()
        while len(self.groups) > 12:
            del self.groups[min(self.groups)]

    def on_joints(self, message):
        self.joints.update(zip(message.name, message.position))

    def wait_for(self, condition, timeout_s, description):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if condition():
                return
        raise TimeoutError(f'Timed out waiting for {description}')

    def run(self, args, result):
        import cv2
        self.wait_for(lambda: self.latest_group is not None, 30., 'same-exposure RGB-D and CameraInfo')
        if not self.client.wait_for_service(timeout_sec=15.):
            raise TimeoutError('/yolo_detection is unavailable')
        group = self.latest_group
        color_message, depth_message, info = group['color'], group['depth'], group['info']
        exposure = stamp_ns(color_message.header.stamp)
        result.update({'exposure_stamp_ns': exposure,
                       'camera_info_stamp_ns': stamp_ns(info.header.stamp),
                       'optical_frame': color_message.header.frame_id,
                       'depth_encoding': depth_message.encoding,
                       'depth_units_per_metre': args.depth_units_per_metre,
                       'rail_station_m': self.joints.get('machine_slide'),
                       'sampled_joints': dict(self.joints)})
        request = YoloDetection.Request(color_image=color_message, depth_image=depth_message)
        future = self.client.call_async(request)
        self.wait_for(future.done, 30., 'GT YOLO response')
        response = future.result()
        if response is None:
            raise RuntimeError('GT YOLO response is empty')
        result['response_images_unchanged'] = (response.color_image == color_message
                                               and response.depth_image == depth_message)
        result['response_aligned'] = (len(response.class_ids) == len(response.class_names)
                                      == len(response.confidences) == len(response.masks)
                                      and len(response.bboxes) == 4 * len(response.masks))
        bridge = TomatoSimCvBridge()
        color = bridge.imgmsg_to_cv2(color_message, 'bgr8')
        depth_counts = bridge.imgmsg_to_cv2(depth_message)
        depth_m = depth_image_metres(depth_message, args.depth_units_per_metre)
        masks = [bridge.imgmsg_to_cv2(mask.mask, 'mono8') for mask in response.masks]
        result['detections'] = [{'class_id': int(response.class_ids[index]),
                                 'class_name': name,
                                 'bbox_xyxy': list(response.bboxes[4*index:4*index+4]),
                                 'mask_pixels': int(np.count_nonzero(masks[index])),
                                 'mask_metadata_aligned': (
                                     response.masks[index].class_names == name
                                     and response.masks[index].class_id == response.class_ids[index])}
                                for index, name in enumerate(response.class_names)]
        args.output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.output / 'rgb.png'), color)
        arrays = {'aligned_depth_m': depth_m, 'depth_counts': depth_counts,
                  'camera_matrix': np.asarray(info.k).reshape((3, 3))}
        for index, name in enumerate(response.class_names):
            arrays[name + '_mask'] = masks[index]
            cv2.imwrite(str(args.output / (name + '_mask.png')), masks[index])
        source_paths = [args.control_source, args.stem_source]
        result['original_source_sha256'] = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
        methods = original_vision_methods(args.control_source, self.get_logger())
        stems, tomatoes, trunks = methods.classify_detections(
            response, len(response.class_ids), len(response.masks))
        matched_image, pairs, rectangles = methods.combine_boxes(color.copy(), stems, tomatoes,
                                                                 color.shape[:2])
        result['original_main_matched_pairs'] = len(pairs)
        result['original_main_matched_rectangles'] = rectangles
        cv2.imwrite(str(args.output / 'matched_boxes.png'), matched_image)
        if not pairs:
            np.savez_compressed(args.output / 'exposure_data.npz', **arrays)
            raise RuntimeError('Original main-controller box matching found no stem/tomato pair')
        transform = self.buffer.lookup_transform(
            'world', color_message.header.frame_id.lstrip('/'),
            Time.from_msg(color_message.header.stamp, clock_type=self.get_clock().clock_type),
            timeout=Duration(seconds=.5)).transform
        world_from_optical = transform_matrix(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        arrays['world_from_optical'] = world_from_optical
        result['world_from_optical'] = world_from_optical.tolist()
        module = types.ModuleType('original_stage2_stem')
        module.__file__ = str(args.stem_source)
        exec(compile(args.stem_source.read_bytes(), str(args.stem_source), 'exec'), module.__dict__)
        calculator = module.StemGrowthAndPoints()
        calculator.save_img = False
        stem, tomato = pairs[0]
        stem_mask, tomato_mask = masks[stem[5]], masks[tomato[5]]
        algorithm, thin, stdout = original_stem_result(
            calculator, stem, tomato, stem_mask, tomato_mask, color)
        (args.output / 'original_stem_algorithm.log').write_text(stdout)
        if not isinstance(thin, np.ndarray):
            raise RuntimeError('Original algorithm did not produce a skeleton array')
        arrays['original_skeleton'] = thin
        cv2.imwrite(str(args.output / 'original_skeleton.png'), (thin > 0).astype(np.uint8) * 255)
        result['original_skeleton_points'] = int(np.count_nonzero(thin))
        result['original_algorithm_return_items'] = len(algorithm)
        result['original_algorithm_success'] = bool(algorithm[0])
        result['growth_direction'] = algorithm[1]
        result['tangent_angle_rad'] = float(algorithm[8])
        result['picking_point_pixel'] = [float(value) for value in algorithm[3]]
        x, y = map(int, algorithm[3])
        result['picking_point_inside_visible_stem_mask'] = bool(stem_mask[y, x])
        result['observed_depth_at_pick_m'] = float(depth_m[y, x])
        result['original_main_after_stop_depth_m'] = float(methods.get_valid_depth_value(
            depth_counts, [], stem_mask, x, y))
        result['original_main_first_seen_mask_average_depth_m'] = float(
            methods.get_mask_average_depth(stem_mask, depth_counts))
        point_optical = np.linalg.inv(arrays['camera_matrix']) @ [x, y, 1.]
        point_optical *= depth_m[y, x]
        scene = json.loads(args.scene_file.read_text())
        result['metric_camera_pick'] = point_error_components(
            point_optical, world_from_optical, scene['cut_point_world'],
            scene['peduncle_tangent_world'])
        result['passed_geometry_and_stem'] = bool(
            result['response_images_unchanged'] and result['response_aligned']
            and result['original_algorithm_success'] and result['original_skeleton_points'] > 10
            and result['picking_point_inside_visible_stem_mask']
            and result['metric_camera_pick']['total_error_m'] < .005)
        main_depth = result['original_main_after_stop_depth_m']
        result['main_after_stop_depth_usable'] = bool(.2 < main_depth < 1.)
        if result['main_after_stop_depth_usable']:
            main_point = np.linalg.inv(arrays['camera_matrix']) @ [x, y, 1.]
            main_point *= main_depth
            result['original_main_pick'] = point_error_components(
                main_point, world_from_optical, scene['cut_point_world'],
                scene['peduncle_tangent_world'])
            transform = self.buffer.lookup_transform(
                'arm_base_link', color_message.header.frame_id.lstrip('/'),
                Time.from_msg(color_message.header.stamp, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=.5)).transform
            base_from_optical = transform_matrix(
                [transform.translation.x, transform.translation.y, transform.translation.z],
                [transform.rotation.x, transform.rotation.y,
                 transform.rotation.z, transform.rotation.w])
            arrays['base_from_optical'] = base_from_optical
            result['base_from_optical'] = base_from_optical.tolist()
            base_points, endpoint_depths = [], []
            for pixel in algorithm[5:8]:
                u, v = map(int, pixel)
                endpoint_depth = float(methods.get_valid_depth_value(
                    depth_counts, [], stem_mask, u, v))
                endpoint_depths.append(endpoint_depth)
                point = np.linalg.inv(arrays['camera_matrix']) @ [u, v, 1.]
                base_points.append(base_from_optical @ np.r_[point * endpoint_depth, 1.])
            result['original_main_endpoint_depths_m'] = endpoint_depths
            if not all(.2 < value < 1. for value in endpoint_depths):
                raise RuntimeError('Original main-controller endpoint depth validation failed')
            base_point = base_from_optical @ np.r_[main_point, 1.]
            methods.first_see_harvestable_tomato = 1
            methods.is_use_base_angle = True
            methods.is_two_stage = False
            methods.gripper_length = args.gripper_length
            built = original_request_builder(args.control_source)(
                methods, *base_point[:3], *base_points, algorithm[1], float(algorithm[8]))
            result['original_main_base_pick_m'] = base_point[:3].tolist()
            result['original_main_base_endpoints_m'] = [point[:3].tolist() for point in base_points]
            result['original_main_request_filtered'] = built is None
            if built is not None:
                request, base_angle, approach_angle = built
                result['original_main_base_angle_rad'] = float(base_angle)
                result['original_main_base_angle_deg'] = math.degrees(base_angle)
                result['original_main_request'] = {
                    'xyz_m': [request.x, request.y, request.z],
                    'rpy_rad': [request.rx, request.ry, request.rz],
                    'rpy_deg': list(np.degrees([request.rx, request.ry, request.rz])),
                    'mid2target_angle_rad': float(approach_angle),
                    'gripper_length_m': args.gripper_length,
                    'start2mid': request.start2mid, 'mid_to_target': request.mid_to_target}
                from tomato_sim_adapters.arm_kinematics import ArmKinematics
                from tomato_sim_adapters.sim_aubo_motion_service import waypoints
                midpoint, target, rotation = waypoints(request, args.gripper_length)
                base_from_requested_tcp = np.array([[rotation[row, column]
                    for column in range(3)] for row in range(3)])
                world_from_base = world_from_optical @ np.linalg.inv(base_from_optical)
                result['original_main_request']['sim_midpoint_base_m'] = midpoint.tolist()
                result['original_main_request']['sim_target_base_m'] = target.tolist()
                result['original_main_request']['sim_tcp_interpretation_crossing_angle_deg'] = (
                    cutter_crossing_angle(world_from_base, base_from_requested_tcp,
                                          scene['peduncle_tangent_world']))
                mounting = ArmKinematics().wrist_to_tcp.M
                flange_from_cutter = np.array([[mounting[row, column]
                    for column in range(3)] for row in range(3)])
                result['original_main_request']['sdk_flange_interpretation_crossing_angle_deg'] = (
                    cutter_crossing_angle(world_from_base,
                                          base_from_requested_tcp @ flange_from_cutter,
                                          scene['peduncle_tangent_world']))
                result['original_main_distance_gate_passed'] = bool(
                    np.linalg.norm(base_point[:3]) <= .784 + args.gripper_length
                    and base_point[2] >= .25)
        np.savez_compressed(args.output / 'exposure_data.npz', **arrays)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=WORKSPACE / 'validation/stage2/vision')
    parser.add_argument('--camera-root', default='/camera')
    parser.add_argument('--depth-units-per-metre', type=float, default=1000.)
    parser.add_argument('--gripper-length', type=float, default=.10)
    parser.add_argument('--scene-file', type=Path,
                        default=WORKSPACE / 'validation/phase3/phase3_scene.json')
    parser.add_argument('--stem-source', type=Path, default=REAL_CONTROL / 'stemGrowthAndPoints.py')
    parser.add_argument('--control-source', type=Path,
                        default=REAL_CONTROL / 'xc_robot_main_control_node.py')
    args, ros_args = parser.parse_known_args()
    args.output.mkdir(parents=True, exist_ok=True)
    result = {'mode': 'camera_gt_service_original_stem_diagnostic',
              'camera_root': args.camera_root, 'passed_geometry_and_stem': False}
    rclpy.init(args=ros_args)
    node = VisionChecks(args.camera_root)
    try:
        node.run(args, result)
    except Exception as error:
        result['error'] = str(error)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        (args.output / 'vision_diagnostic.json').write_text(
            json.dumps(result, indent=2, allow_nan=False) + '\n')
        print(json.dumps(result, indent=2, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
