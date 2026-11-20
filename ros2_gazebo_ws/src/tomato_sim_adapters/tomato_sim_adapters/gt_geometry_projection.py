"""SDF visual geometry, pinhole surface rasterization, and depth visibility.

Depth values are optical-axis z in metres. Pixel rays pass through the integer
coordinates used by ROS CameraInfo and the downstream deprojection algorithm.
"""
from dataclasses import dataclass
import itertools
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

CLASS_NAMES = ('stem', 'tomato', 'trunk')
NEAR_PLANE_M = 0.0001


@dataclass
class Geometry:
    class_name: str
    kind: str
    world_from_local: np.ndarray
    radius: float = 0.
    length: float = 0.
    vertices: np.ndarray = None
    faces: np.ndarray = None
    name: str = ''


@dataclass
class Detection:
    class_id: int
    class_name: str
    mask: np.ndarray
    bbox: list
    projected_pixels: int
    max_depth_residual_m: float
    confidence: float = 1.


def pose_matrix(text=None):
    """Construct parent-from-child from an SDF xyz roll/pitch/yaw pose."""
    x, y, z, roll, pitch, yaw = map(float, (text or '0 0 0 0 0 0').split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = [x, y, z]
    return result


def transform_matrix(translation, quaternion):
    """Construct target-from-source from a ROS xyzw quaternion and translation."""
    q = np.asarray(quaternion, dtype=float)
    norm = float(q @ q)
    if norm < 1e-20:
        raise ValueError('Transform quaternion has zero norm')
    x, y, z, w = q / math.sqrt(norm)
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    result[:3, 3] = translation
    return result


def _sdf_pose(element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('SDF relative_to poses require frame resolution')
    return pose_matrix(None if pose is None else pose.text)


def _resolve_uri(uri, model_paths, relative_to):
    if uri.startswith('model://'):
        tail = uri[len('model://'):]
        candidates = [root / tail for root in model_paths]
    elif uri.startswith('file://'):
        candidates = [Path(uri[len('file://'):])]
    else:
        candidates = [relative_to / uri]
    for path in candidates:
        if path.is_file():
            return path
        if path.is_dir() and (path / 'model.sdf').is_file():
            return path / 'model.sdf'
    raise FileNotFoundError(f'Cannot resolve geometry URI {uri} in {model_paths}')


def _read_obj(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(value) for value in fields[1:4]])
        elif fields[0] == 'f':
            indices = [int(value.split('/')[0]) for value in fields[1:]]
            indices = [index - 1 if index > 0 else len(vertices) + index for index in indices]
            faces.extend((indices[0], indices[index], indices[index + 1])
                         for index in range(1, len(indices) - 1))
    vertices = np.asarray(vertices, dtype=float).reshape((-1, 3))
    faces = np.asarray(faces, dtype=int).reshape((-1, 3))
    if not len(vertices) or not len(faces):
        raise ValueError(f'OBJ has no triangle geometry: {path}')
    if np.any(faces < 0) or np.any(faces >= len(vertices)):
        raise ValueError(f'OBJ face index is out of range: {path}')
    return vertices, faces


def load_scene_geometry(scene_file, model_paths=()):
    """Load target peduncle, fruit, and plant trunk from generated SDF visuals."""
    scene_path = Path(scene_file).resolve()
    scene = json.loads(scene_path.read_text())
    world_path = Path(scene['world'])
    if not world_path.is_absolute():
        world_path = scene_path.parent / world_path
    search_paths = [scene_path.parent / 'models', world_path.parent / 'models']
    search_paths.extend(Path(value) for value in model_paths)
    search_paths.extend(Path(value) for value in os.environ.get('GAZEBO_MODEL_PATH', '').split(':')
                        if value)
    search_paths.extend(parent / 'gazebo_migration/assets' for parent in scene_path.parents)
    search_paths = list(dict.fromkeys(search_paths))
    world = ET.parse(world_path).getroot().find('world')
    geometry = []
    for model_name in (scene['plant_model'], scene['target_model']):
        include = next((item for item in world.findall('include')
                        if item.findtext('name') == model_name), None)
        if include is None:
            raise ValueError(f'Scene model {model_name} is absent from {world_path}')
        model_path = _resolve_uri(include.findtext('uri'), search_paths, world_path.parent)
        model = ET.parse(model_path).getroot().find('model')
        world_from_model = _sdf_pose(include) @ _sdf_pose(model)
        for link in model.findall('link'):
            world_from_link = world_from_model @ _sdf_pose(link)
            for visual in link.findall('visual'):
                name = visual.get('name', '')
                if name.startswith(('contact_stalk_visual_', 'contact_stalk_joint_')):
                    class_name = 'stem'
                elif model_name == scene['plant_model'] and name.endswith('_vine_visual'):
                    class_name = 'trunk'
                elif model_name == scene['target_model'] and name.endswith('_mesh_visual'):
                    class_name = 'tomato'
                else:
                    continue
                shape = visual.find('geometry')
                world_from_visual = world_from_link @ _sdf_pose(visual)
                if shape.find('mesh') is not None:
                    mesh = shape.find('mesh')
                    mesh_path = _resolve_uri(mesh.findtext('uri'), search_paths, model_path.parent)
                    if mesh_path.suffix.lower() != '.obj':
                        raise ValueError(f'Expected OBJ visual geometry, got {mesh_path}')
                    vertices, faces = _read_obj(mesh_path)
                    scale = np.array(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                    item = Geometry(class_name, 'mesh', world_from_visual,
                                    vertices=vertices * scale, faces=faces, name=name)
                elif shape.find('sphere') is not None:
                    item = Geometry(class_name, 'sphere', world_from_visual,
                                    radius=float(shape.findtext('sphere/radius')), name=name)
                elif shape.find('cylinder') is not None:
                    item = Geometry(class_name, 'cylinder', world_from_visual,
                                    radius=float(shape.findtext('cylinder/radius')),
                                    length=float(shape.findtext('cylinder/length')), name=name)
                else:
                    raise ValueError(f'Unsupported visual geometry for {name}')
                geometry.append(item)
    missing = set(CLASS_NAMES) - {item.class_name for item in geometry}
    if missing:
        raise ValueError(f'Scene is missing visual classes: {sorted(missing)}')
    return geometry


def _pixel_bounds(points, camera, width, height):
    if np.max(points[:, 2]) < NEAR_PLANE_M:
        return None
    if np.min(points[:, 2]) < NEAR_PLANE_M:
        return 0, 0, width, height
    projected = points @ camera.T
    projected = projected[:, :2] / projected[:, 2:3]
    lower = np.maximum(np.floor(np.min(projected, axis=0)), [0, 0])
    upper = np.minimum(np.ceil(np.max(projected, axis=0)) + 1, [width, height])
    x1, y1 = lower.astype(int)
    x2, y2 = upper.astype(int)
    return (x1, y1, x2, y2) if x1 < x2 and y1 < y2 else None


def _primitive_depth(item, camera_from_local, camera, width, height, depth):
    radius = item.radius
    half_length = item.length / 2 if item.kind == 'cylinder' else radius
    bounds = np.array(list(itertools.product((-radius, radius), (-radius, radius),
                                            (-half_length, half_length))))
    bounds = bounds @ camera_from_local[:3, :3].T + camera_from_local[:3, 3]
    roi = _pixel_bounds(bounds, camera, width, height)
    if roi is None:
        return
    x1, y1, x2, y2 = roi
    v, u = np.mgrid[y1:y2, x1:x2]
    rays = np.stack((u, v, np.ones_like(u)), axis=-1) @ np.linalg.inv(camera).T
    directions = rays @ camera_from_local[:3, :3]
    origin = -camera_from_local[:3, :3].T @ camera_from_local[:3, 3]
    dimensions = 3 if item.kind == 'sphere' else 2
    a = np.sum(directions[..., :dimensions] ** 2, axis=-1)
    b = np.sum(directions[..., :dimensions] * origin[:dimensions], axis=-1)
    c = np.sum(origin[:dimensions] ** 2) - radius ** 2
    discriminant = b ** 2 - a * c
    root = np.sqrt(np.maximum(discriminant, 0))
    surface = np.full(u.shape, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for sign in (-1, 1):
            z = (-b + sign * root) / a
            valid = (discriminant >= 0) & (a > 1e-20) & (z >= NEAR_PLANE_M)
            if item.kind == 'cylinder':
                valid &= np.abs(origin[2] + z * directions[..., 2]) <= half_length
            np.minimum(surface, np.where(valid, z, np.inf), out=surface)
        if item.kind == 'cylinder':
            for cap in (-half_length, half_length):
                z = (cap - origin[2]) / directions[..., 2]
                cap_points = origin[:2] + z[..., None] * directions[..., :2]
                valid = ((z >= NEAR_PLANE_M) & (np.abs(directions[..., 2]) > 1e-20)
                         & (np.sum(cap_points ** 2, axis=-1) <= radius ** 2))
                np.minimum(surface, np.where(valid, z, np.inf), out=surface)
    np.minimum(depth[y1:y2, x1:x2], surface, out=depth[y1:y2, x1:x2])


def _clip_triangle(triangle):
    polygon = []
    previous = triangle[-1]
    for point in triangle:
        inside, previous_inside = point[2] >= NEAR_PLANE_M, previous[2] >= NEAR_PLANE_M
        if inside != previous_inside:
            fraction = (NEAR_PLANE_M - previous[2]) / (point[2] - previous[2])
            polygon.append(previous + fraction * (point - previous))
        if inside:
            polygon.append(point)
        previous = point
    return [np.array([polygon[0], polygon[index], polygon[index + 1]])
            for index in range(1, len(polygon) - 1)]


def _raster_triangle(triangle, camera, width, height, depth):
    roi = _pixel_bounds(triangle, camera, width, height)
    if roi is None:
        return
    projected = triangle @ camera.T
    screen = projected[:, :2] / projected[:, 2:3]
    ax, ay = screen[0]
    bx, by = screen[1]
    cx, cy = screen[2]
    denominator = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
    if abs(denominator) < 1e-14:
        return
    x1, y1, x2, y2 = roi
    v, u = np.mgrid[y1:y2, x1:x2]
    first = ((by - cy) * (u - cx) + (cx - bx) * (v - cy)) / denominator
    second = ((cy - ay) * (u - cx) + (ax - cx) * (v - cy)) / denominator
    third = 1 - first - second
    inside = (first >= -1e-9) & (second >= -1e-9) & (third >= -1e-9)
    inverse_depth = first / triangle[0, 2] + second / triangle[1, 2] + third / triangle[2, 2]
    surface = np.full(u.shape, np.inf)
    np.divide(1., inverse_depth, out=surface, where=inside & (inverse_depth > 0))
    np.minimum(depth[y1:y2, x1:x2], surface, out=depth[y1:y2, x1:x2])


def render_depths(geometry, optical_from_world, camera, width, height):
    """Rasterize the nearest analytic/triangle surface for each semantic class."""
    camera = np.asarray(camera, dtype=float).reshape((3, 3))
    if width <= 0 or height <= 0 or camera[0, 0] <= 0 or camera[1, 1] <= 0:
        raise ValueError('Camera image dimensions and focal lengths must be positive')
    depths = {name: np.full((height, width), np.inf) for name in CLASS_NAMES}
    for item in geometry:
        camera_from_local = optical_from_world @ item.world_from_local
        depth = depths[item.class_name]
        if item.kind in ('sphere', 'cylinder'):
            _primitive_depth(item, camera_from_local, camera, width, height, depth)
        elif item.kind == 'mesh':
            vertices = item.vertices @ camera_from_local[:3, :3].T + camera_from_local[:3, 3]
            if _pixel_bounds(vertices, camera, width, height) is None:
                continue
            for face in item.faces:
                triangle = vertices[face]
                if np.all(triangle[:, 2] >= NEAR_PLANE_M):
                    _raster_triangle(triangle, camera, width, height, depth)
                elif np.any(triangle[:, 2] >= NEAR_PLANE_M):
                    for clipped in _clip_triangle(triangle):
                        _raster_triangle(clipped, camera, width, height, depth)
        else:
            raise ValueError(f'Unsupported geometry kind {item.kind}')
    return depths


def visible_detections(depths, observed_depth_m, tolerance_m=.012):
    """Return only visible instances with one aligned mask and box per detection."""
    if not np.isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError('Depth consistency tolerance must be finite and nonnegative')
    observed = np.asarray(observed_depth_m)
    nearest = np.minimum.reduce(list(depths.values()))
    valid_observed = np.isfinite(observed) & (observed > 0)
    detections = []
    for class_id, class_name in enumerate(CLASS_NAMES):
        if class_name not in depths:
            continue
        projected = depths[class_name]
        if projected.shape != observed.shape:
            raise ValueError('Projected and aligned depth image dimensions differ')
        finite = np.isfinite(projected)
        residual = np.full(projected.shape, np.inf)
        np.subtract(projected, observed, out=residual, where=finite & valid_observed)
        residual = np.abs(residual)
        visible = finite & valid_observed & (residual <= tolerance_m) & (projected <= nearest + 1e-6)
        y, x = np.nonzero(visible)
        if not len(x):
            continue
        detections.append(Detection(
            class_id, class_name, visible.astype(np.uint8) * 255,
            [float(x.min()), float(y.min()), float(x.max() + 1), float(y.max() + 1)],
            int(np.count_nonzero(finite)), float(np.max(residual[visible]))))
    return detections
