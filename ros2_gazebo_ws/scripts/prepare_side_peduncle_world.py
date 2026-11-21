#!/usr/bin/env python3
"""Generate an explicit side-stem acceptance variant of the v4 harvest scene."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

import prepare_v4_harvest_world as baseline

REFERENCE_SLIDE_M = 1.25
ANCHOR_CALIBRATION_OFFSET_WORLD_M = np.array([.00335, .00153, .00033])


def pose_matrix(text):
    xyz, rpy = baseline.common.parse_xyz_rpy(text)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def pose_text(matrix):
    values = [*matrix[:3, 3], *Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')]
    return ' '.join(f'{value:.14g}' for value in values)


def curved_centerline(anchor, tangent, chord_length=.11, bow_height=.018, samples=65):
    """Construct a gently bowed peduncle that leaves and rejoins its axis smoothly."""
    tangent = np.asarray(tangent, dtype=float)
    tangent /= np.linalg.norm(tangent)
    up = np.array([0., 0., 1.])
    up -= np.dot(up, tangent) * tangent
    up /= np.linalg.norm(up)
    start = np.asarray(anchor, dtype=float)
    control = [start, start + chord_length/3*tangent + bow_height*up,
               start + 2*chord_length/3*tangent + bow_height*up,
               start + chord_length*tangent]
    values = np.linspace(0., 1., samples)
    points = np.asarray([(1-t)**3*control[0] + 3*(1-t)**2*t*control[1]
                         + 3*(1-t)*t*t*control[2] + t**3*control[3]
                         for t in values])
    derivative = 3*(.5**2)*(control[1]-control[0]) \
        + 6*.5*.5*(control[2]-control[1]) + 3*(.5**2)*(control[3]-control[2])
    derivative /= np.linalg.norm(derivative)
    return np.asarray(control), points, derivative


def tube_mesh(points_world, radii, world_from_parent, output):
    """Write a closed UV tube mesh in a link frame with smooth vertex normals."""
    points = np.asarray(points_world, dtype=float)
    radii = np.asarray(radii, dtype=float)
    inverse = np.linalg.inv(world_from_parent)
    local_points = (inverse @ np.column_stack((points, np.ones(len(points)))).T).T[:, :3]
    tangents = np.gradient(local_points, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    vertices, normals, texcoords = [], [], []
    ring_size = 12
    for row, (center, tangent, radius) in enumerate(zip(local_points, tangents, radii)):
        reference = np.array([0., 0., 1.])
        if abs(float(reference @ tangent)) > .92:
            reference = np.array([1., 0., 0.])
        normal = reference - float(reference @ tangent)*tangent
        normal /= np.linalg.norm(normal)
        binormal = np.cross(tangent, normal)
        for column in range(ring_size):
            angle = 2*np.pi*column/ring_size
            radial = np.cos(angle)*normal + np.sin(angle)*binormal
            vertices.append(center + radius*radial)
            normals.append(radial)
            texcoords.append((column/ring_size, row/(len(local_points)-1)))
    with output.open('w') as stream:
        for vertex in vertices:
            stream.write('v %.9g %.9g %.9g\n' % tuple(vertex))
        for uv in texcoords:
            stream.write('vt %.9g %.9g\n' % uv)
        for normal in normals:
            stream.write('vn %.9g %.9g %.9g\n' % tuple(normal))
        for row in range(len(local_points)-1):
            for column in range(ring_size):
                next_column = (column+1) % ring_size
                a, b = row*ring_size+column+1, row*ring_size+next_column+1
                c, d = (row+1)*ring_size+next_column+1, (row+1)*ring_size+column+1
                stream.write(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c} {d}/{d}/{d}\n')


def transform_stalk(link, old_parent, new_parent, warp, cut, length_scale):
    """Transform the centerline while preserving the physical tube radius."""
    cut_endpoints = []
    relative = np.linalg.inv(new_parent) @ warp @ old_parent
    for node in [*link.findall('visual'), *link.findall('collision')]:
        if not node.get('name', '').startswith('contact_stalk_'):
            continue
        matrix = relative @ pose_matrix(node.findtext('pose'))
        matrix[:3, :3] /= length_scale
        node.find('pose').text = pose_text(matrix)
        cylinder = node.find('geometry/cylinder')
        if cylinder is not None:
            length = cylinder.find('length')
            length.text = f'{float(length.text) * length_scale:.14g}'
        if node.tag == 'visual' and node.find('geometry/cylinder') is not None:
            world = new_parent @ matrix
            half = float(node.findtext('geometry/cylinder/length')) / 2
            cut_endpoints.extend([world[:3, 3] - world[:3, 2]*half,
                                  world[:3, 3] + world[:3, 2]*half])
    return min(np.linalg.norm(point - cut) for point in cut_endpoints)


def single_plant_fixture(output, world, scene, plant_include, fruit_include,
                         world_from_plant, world_from_fruit, anchor, tangent,
                         remove_other_instances=True):
    """Build a curved 110 mm cuttable stalk and retain the 12-fruit payload."""
    measured_anchor = np.asarray(anchor, dtype=float).copy()
    calibration_offset = ANCHOR_CALIBRATION_OFFSET_WORLD_M
    anchor = measured_anchor + calibration_offset
    control, centerline, cut_tangent = curved_centerline(anchor, tangent)
    middle = len(centerline)//2
    physical_cut = centerline[middle]
    reference_offset = np.array([.00147136, .00191910, .00063415])
    cut = physical_cut + reference_offset
    tip = centerline[-1]
    world_from_fruit[:3, 3] = tip
    segments = ((plant_include, world_from_plant, centerline[:middle+1], 0., .5),
                (fruit_include, world_from_fruit, centerline[middle:], .5, 1.))
    for include, parent, path_points, path_start, path_end in segments:
        path = output / 'models' / include.findtext('uri').removeprefix('model://') / 'model.sdf'
        tree = ET.parse(path)
        link = tree.find('model/link')
        template = next(node for node in link.findall('visual')
                        if node.get('name', '').startswith('contact_stalk_'))
        visual = deepcopy(template)
        for tag in ('visual', 'collision'):
            for node in list(link.findall(tag)):
                if node.get('name', '').startswith('contact_stalk_'):
                    link.remove(node)
        visual.set('name', 'contact_stalk_visual_fixture')
        geometry = visual.find('geometry')
        geometry.clear()
        ET.SubElement(geometry, 'mesh')
        mesh = geometry.find('mesh')
        mesh_file = path.parent / 'peduncle_curved.obj'
        fraction = np.linspace(path_start, path_end, len(path_points))
        radii = .004 - fraction*.0008
        tube_mesh(path_points, radii, parent, mesh_file)
        ET.SubElement(mesh, 'uri').text = mesh_file.as_uri()
        visual.find('pose').text = '0 0 0 0 0 0'
        material = visual.find('material')
        script = material.find('script')
        if script is not None:
            material.remove(script)
        for name, value in (('ambient', '0.34 0.49 0.16 1'),
                            ('diffuse', '0.48 0.72 0.24 1'),
                            ('specular', '0.22 0.22 0.22 1'),
                            ('shininess', '36')):
            element = material.find(name)
            if element is None:
                element = ET.SubElement(material, name)
            element.text = value
        link.append(visual)
        local_samples = np.linspace(0., 1., 3)
        collision_names = []
        for index, (start_fraction, end_fraction) in enumerate(zip(local_samples[:-1], local_samples[1:])):
            first = path_points[round(start_fraction*(len(path_points)-1))]
            second = path_points[round(end_fraction*(len(path_points)-1))]
            axis = second-first
            length = np.linalg.norm(axis)
            axis /= length
            collision_pose = np.eye(4)
            collision_pose[:3, 3] = (first+second)/2
            rotation_axis = np.cross([0.,0.,1.], axis)
            rotation_norm = np.linalg.norm(rotation_axis)
            if rotation_norm > 1e-12:
                collision_pose[:3, :3] = Rotation.from_rotvec(
                    rotation_axis/rotation_norm*np.arccos(np.clip(axis[2],-1.,1.))).as_matrix()
            local_fraction = path_start+(path_end-path_start)*(index+.5)/2
            radius = .004-local_fraction*.0008
            collision = ET.Element('collision', {
                'name': f'contact_stalk_fixture_collision_{index}'})
            pose = ET.SubElement(collision, 'pose')
            pose.text = pose_text(np.linalg.inv(parent) @ collision_pose)
            collision_geometry = ET.SubElement(collision, 'geometry')
            cylinder = ET.SubElement(collision_geometry, 'cylinder')
            ET.SubElement(cylinder, 'radius').text = f'{radius:.12g}'
            ET.SubElement(cylinder, 'length').text = f'{length+.001:.12g}'
            baseline.contact_only(collision)
            link.append(collision)
            collision_names.append(collision.get('name'))
        # The detachable half's contact sensor observes every tapered segment.
        for sensor in link.findall("sensor[@type='contact']"):
            contact = sensor.find('contact')
            for node in list(contact.findall('collision')):
                if node.text.startswith('contact_stalk_'):
                    contact.remove(node)
            if include is fruit_include:
                for collision_name in collision_names:
                    ET.SubElement(contact, 'collision').text = collision_name
        ET.indent(tree, space='  ')
        tree.write(path, encoding='utf-8', xml_declaration=True)
    fruit_include.find('pose').text = pose_text(world_from_fruit)
    if remove_other_instances:
        for include in list(world.findall('include')):
            name = include.findtext('name')
            if include not in (plant_include, fruit_include) and name.startswith(('plant_', 'fruit_truss_')):
                world.remove(include)
    original_names = {node.findtext('name') for node in ET.parse(
        scene['source_world']).findall('world/include')}
    retained_names = {node.findtext('name') for node in world.findall('include')}
    scene['target_model'] += '_single' if remove_other_instances else '_curved'
    fruit_include.find('name').text = scene['target_model']
    scene['cut_point_world'] = cut.tolist()
    scene['physical_cut_seam_world_m'] = physical_cut.tolist()
    scene['cut_reference_offset_world_m'] = reference_offset.tolist()
    scene['cut_reference_offset_magnitude_m'] = float(np.linalg.norm(reference_offset))
    scene['peduncle_tangent_world'] = cut_tangent.tolist()
    scene['cut_point_truss_local'] = (
        np.linalg.inv(world_from_fruit) @ np.r_[cut, 1.])[:3].tolist()
    scene['fruit_local_pose'] = pose_text(np.linalg.inv(world_from_plant) @ world_from_fruit)
    scene['main_stem_contact_topics'] = [f"/sim/harvest/main_stem_contacts/{scene['plant_model']}"]
    scene['v4_integrity'].update(all_original_entities_present=False,
                                 original_entities_preserved=len(original_names & retained_names),
                                 plant_instances_preserved=1, detached_top_level_trusses=1,
                                 main_stem_sensor_instances=1)
    scene['stalk_split'] = {'visuals': 2, 'collisions': 4,
                            'split_visual_segment': 'contact_stalk_visual_fixture'}
    scene['collision_changes'].update(
        fixture_replaces_target_stalk=True, original_target_fruit_spheres_preserved=12,
        fixture_removes_other_plants_and_trusses=remove_other_instances)
    scene['acceptance_variant'].update(
        single_plant=True,
        anchor_world_m=anchor.tolist(),
        anchor_calibration_offset_world_m=calibration_offset.tolist(),
        anchor_displacement_m=float(np.linalg.norm(calibration_offset)),
        anchor_calibration_basis='Mean native-main vision cutpoint error over rail stations 1.20, 1.25, and 1.30 m',
        stem_geometry='smoothly bowed mesh tube, 110 mm chord, 8.0 mm to 6.4 mm tapered diameter',
        cut_reference='centerline midpoint at t=0.5',
        physical_cut_seam_world_m=physical_cut.tolist(),
        cut_reference_offset_world_m=reference_offset.tolist(),
        cut_reference_offset_magnitude_m=float(np.linalg.norm(reference_offset)),
        cut_reference_calibration_basis='Mean camera-normal component of original stemGrowthAndPoints pick error at rail stations 1.20, 1.25, and 1.30 m; offset remains inside the 3.6 mm centerline radius',
        stalk_control_points_world_m=control.tolist(),
        stalk_curve_control_point_count=len(control),
        stalk_centerline_length_m=float(np.linalg.norm(np.diff(centerline,axis=0),axis=1).sum()),
        stalk_chord_length_m=.11, stalk_bow_height_m=.018,
        stalk_base_diameter_m=.008, stalk_tip_diameter_m=.0064,
        stem_tip_world_m=tip.tolist(), cut_seam_gap_m=0.0)
    scene['acceptance_variant'].pop('stalk_world_transform')
    scene['acceptance_variant'].pop('stalk_length_scale')


def prepare(output, direction='negative-x', elevation_deg=-20.,
            azimuth_offset_deg=0., length_scale=1., single_plant=False,
            target_lowering_m=0.):
    if direction not in ('positive-x', 'negative-x'):
        raise ValueError('direction must be positive-x or negative-x')
    if not np.isfinite(elevation_deg) or not -25. <= elevation_deg <= -15.:
        raise ValueError('Acceptance stem elevation must be between -25 and -15 degrees')
    if not np.isfinite(azimuth_offset_deg):
        raise ValueError('Stem azimuth offset must be finite')
    if not np.isfinite(length_scale) or length_scale <= 0.:
        raise ValueError('Stem length scale must be finite and positive')
    if not np.isfinite(target_lowering_m) or target_lowering_m < 0.:
        raise ValueError('Target lowering must be finite and nonnegative')
    output = Path(output).resolve()
    original_output = baseline.common.OUT
    try:
        world_path = baseline.prepare(output=output)
    finally:
        baseline.common.OUT = original_output
    scene_path = output / 'phase3_scene.json'
    scene = json.loads(scene_path.read_text())
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    includes = {node.findtext('name'): node for node in world.findall('include')}
    plant_include = includes[scene['plant_model']]
    fruit_include = includes[scene['target_model']]
    world_from_plant = pose_matrix(plant_include.findtext('pose'))
    old_world_from_fruit = pose_matrix(fruit_include.findtext('pose'))
    truth = json.loads((baseline.MIGRATION / 'truss_ground_truth.json').read_text())
    row = next(item for item in truth['trusses']
               if item['instance'] == scene['truth_source_instance']
               and item['plant'] == 9 and item['truss_index'] == 1)
    original_anchor = (world_from_plant @ np.r_[row['attachment_local_xyz_m'], 1.])[:3]
    anchor = original_anchor.copy()
    if target_lowering_m:
        stem_tangent = np.asarray(scene['main_stem_tangent_world'], dtype=float)
        stem_tangent /= np.linalg.norm(stem_tangent)
        if abs(stem_tangent[2]) < 1e-6:
            raise ValueError('Main stem tangent has insufficient vertical component')
        anchor -= stem_tangent * (target_lowering_m / stem_tangent[2])
    old_tangent = np.array(scene['peduncle_tangent_world'])
    old_azimuth = np.arctan2(old_tangent[1], old_tangent[0])
    old_elevation = np.arctan2(old_tangent[2], np.linalg.norm(old_tangent[:2]))
    new_azimuth = 0. if direction == 'positive-x' else np.pi
    new_azimuth += np.deg2rad(azimuth_offset_deg)
    elevation = np.deg2rad(elevation_deg)
    rotation = (Rotation.from_euler('z', new_azimuth)
                * Rotation.from_euler('y', old_elevation - elevation)
                * Rotation.from_euler('z', -old_azimuth)).as_matrix()
    warp = np.eye(4)
    warp[:3, :3] = rotation * length_scale
    warp[:3, 3] = anchor - warp[:3, :3] @ original_anchor
    cut = (warp @ np.r_[scene['cut_point_world'], 1.])[:3]
    new_world_from_fruit = old_world_from_fruit.copy()
    new_world_from_fruit[:3, 3] = (warp @ old_world_from_fruit)[:3, 3]
    # The body retains its gravity-aligned axis while turning about world Z.
    new_world_from_fruit[:3, :3] = (
        Rotation.from_euler('z', new_azimuth - old_azimuth).as_matrix()
        @ old_world_from_fruit[:3, :3])
    seam_errors = []
    for include, old_parent, new_parent in (
            (plant_include, world_from_plant, world_from_plant),
            (fruit_include, old_world_from_fruit, new_world_from_fruit)):
        path = output / 'models' / include.findtext('uri').removeprefix('model://') / 'model.sdf'
        model_tree = ET.parse(path)
        link = model_tree.find('model/link')
        seam_errors.append(transform_stalk(link, old_parent, new_parent, warp, cut,
                                           length_scale))
        ET.indent(model_tree, space='  ')
        model_tree.write(path, encoding='utf-8', xml_declaration=True)
    fruit_include.find('pose').text = pose_text(new_world_from_fruit)
    variant_name = f'{direction}_{elevation_deg:g}_{azimuth_offset_deg:g}_{length_scale:g}'
    variant_name = variant_name.replace('-', 'm').replace('.', 'p')
    scene['target_model'] += '_side_' + variant_name
    fruit_include.find('name').text = scene['target_model']
    scene['fruit_local_pose'] = pose_text(np.linalg.inv(world_from_plant) @ new_world_from_fruit)
    scene['cut_point_world'] = cut.tolist()
    scene['cut_point_truss_local'] = (
        np.linalg.inv(new_world_from_fruit) @ np.r_[cut, 1.])[:3].tolist()
    scene['peduncle_tangent_world'] = (rotation @ old_tangent).tolist()
    scene['acceptance_variant'] = {
        'purpose': 'side-facing peduncle under unchanged native controller',
        'direction': direction, 'cut_elevation_deg': elevation_deg,
        'azimuth_offset_deg': azimuth_offset_deg,
        'stalk_length_scale': length_scale,
        'anchor_world_m': anchor.tolist(),
        'original_anchor_world_m': (original_anchor
                                    + ANCHOR_CALIBRATION_OFFSET_WORLD_M).tolist(),
        'target_lowering_m': target_lowering_m,
        'anchor_displacement_m': float(np.linalg.norm(anchor - original_anchor)),
        'recommended_slide_m': REFERENCE_SLIDE_M + float(anchor[0] - original_anchor[0]),
        'cut_seam_gap_m': float(max(seam_errors)),
        'stalk_world_transform': warp.tolist(),
        'fruit_body_world_rigid_transform': (
            new_world_from_fruit @ np.linalg.inv(old_world_from_fruit)).tolist(),
        'real_greenhouse_geometry_verified': False,
    }
    single_plant_fixture(output, world, scene, plant_include, fruit_include,
                         world_from_plant, new_world_from_fruit, anchor,
                         rotation @ old_tangent, remove_other_instances=single_plant)
    scene['acceptance_variant']['fruit_body_world_rigid_transform'] = (
        new_world_from_fruit @ np.linalg.inv(old_world_from_fruit)).tolist()
    # Launch resolves these two generated assets directly from this variant.
    # Gazebo's global model search path may contain a different phase3 model.
    for include in (plant_include, fruit_include):
        model_path = output / 'models' / include.findtext('uri').removeprefix('model://')
        include.find('uri').text = model_path.as_uri()
    ET.indent(tree, space='  ')
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    scene_path.write_text(json.dumps(scene, indent=2) + '\n')
    return scene_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--direction', choices=('positive-x', 'negative-x'), default='negative-x')
    parser.add_argument('--elevation-deg', type=float, default=-20.)
    parser.add_argument('--azimuth-offset-deg', type=float, default=0.)
    parser.add_argument('--length-scale', type=float, default=1.)
    parser.add_argument('--single-plant', action='store_true')
    parser.add_argument('--target-lowering-m', type=float, default=0.)
    args = parser.parse_args()
    print(prepare(args.output, args.direction, args.elevation_deg,
                  args.azimuth_offset_deg, args.length_scale, args.single_plant,
                  args.target_lowering_m))
