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


def pose_matrix(text):
    xyz, rpy = baseline.common.parse_xyz_rpy(text)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def pose_text(matrix):
    values = [*matrix[:3, 3], *Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')]
    return ' '.join(f'{value:.14g}' for value in values)


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
                         world_from_plant, world_from_fruit, anchor, tangent):
    """Use one straight 240 mm stalk with its cut reference at the midpoint."""
    cut = anchor + .12 * tangent
    tip = anchor + .24 * tangent
    world_from_fruit[:3, 3] = tip
    endpoints = ((anchor, cut), (cut, tip))
    for include, parent, (start, end) in zip(
            (plant_include, fruit_include), (world_from_plant, world_from_fruit), endpoints):
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
        stalk = np.eye(4)
        stalk[:3, 3] = (start + end) / 2
        axis = np.cross([0, 0, 1], tangent)
        stalk[:3, :3] = Rotation.from_rotvec(
            axis / np.linalg.norm(axis) * np.arccos(tangent[2])).as_matrix()
        visual.set('name', 'contact_stalk_visual_fixture')
        visual.find('pose').text = pose_text(np.linalg.inv(parent) @ stalk)
        geometry = visual.find('geometry')
        geometry.clear()
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = '0.0025'
        ET.SubElement(cylinder, 'length').text = '0.12'
        collision = ET.Element('collision', {'name': 'contact_stalk_fixture_collision'})
        collision.append(deepcopy(visual.find('pose')))
        collision.append(deepcopy(geometry))
        baseline.contact_only(collision)
        link.extend((visual, collision))
        # The detachable half's contact sensor must reference the replacement.
        for sensor in link.findall("sensor[@type='contact']"):
            contact = sensor.find('contact')
            for node in list(contact.findall('collision')):
                if node.text.startswith('contact_stalk_'):
                    contact.remove(node)
            if include is fruit_include:
                ET.SubElement(contact, 'collision').text = collision.get('name')
        ET.indent(tree, space='  ')
        tree.write(path, encoding='utf-8', xml_declaration=True)
    fruit_include.find('pose').text = pose_text(world_from_fruit)
    for include in list(world.findall('include')):
        name = include.findtext('name')
        if include not in (plant_include, fruit_include) and name.startswith(('plant_', 'fruit_truss_')):
            world.remove(include)
    original_names = {node.findtext('name') for node in ET.parse(
        scene['source_world']).findall('world/include')}
    retained_names = {node.findtext('name') for node in world.findall('include')}
    scene['target_model'] += '_single'
    fruit_include.find('name').text = scene['target_model']
    scene['cut_point_world'] = cut.tolist()
    scene['cut_point_truss_local'] = (
        np.linalg.inv(world_from_fruit) @ np.r_[cut, 1.])[:3].tolist()
    scene['fruit_local_pose'] = pose_text(np.linalg.inv(world_from_plant) @ world_from_fruit)
    scene['main_stem_contact_topics'] = [f"/sim/harvest/main_stem_contacts/{scene['plant_model']}"]
    scene['v4_integrity'].update(all_original_entities_present=False,
                                 original_entities_preserved=len(original_names & retained_names),
                                 plant_instances_preserved=1, detached_top_level_trusses=1,
                                 main_stem_sensor_instances=1)
    scene['stalk_split'] = {'visuals': 2, 'collisions': 2,
                            'split_visual_segment': 'contact_stalk_visual_fixture'}
    scene['collision_changes'].update(
        fixture_replaces_target_stalk=True, original_target_fruit_spheres_preserved=12,
        fixture_removes_other_plants_and_trusses=True)
    scene['acceptance_variant'].update(
        single_plant=True, stem_geometry='straight cylinder, radius 2.5 mm, length 240 mm',
        cut_reference='geometric midpoint of the complete stalk',
        stem_tip_world_m=tip.tolist(), cut_seam_gap_m=0.0)
    scene['acceptance_variant'].pop('stalk_world_transform')
    scene['acceptance_variant'].pop('stalk_length_scale')


def prepare(output, direction='negative-x', elevation_deg=-20.,
            azimuth_offset_deg=0., length_scale=1., single_plant=False):
    if direction not in ('positive-x', 'negative-x'):
        raise ValueError('direction must be positive-x or negative-x')
    if not np.isfinite(elevation_deg) or not -25. <= elevation_deg <= -15.:
        raise ValueError('Acceptance stem elevation must be between -25 and -15 degrees')
    if not np.isfinite(azimuth_offset_deg):
        raise ValueError('Stem azimuth offset must be finite')
    if not np.isfinite(length_scale) or length_scale <= 0.:
        raise ValueError('Stem length scale must be finite and positive')
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
    anchor = (world_from_plant @ np.r_[row['attachment_local_xyz_m'], 1.])[:3]
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
    warp[:3, 3] = anchor - warp[:3, :3] @ anchor
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
        'anchor_displacement_m': float(np.linalg.norm((warp @ np.r_[anchor, 1.])[:3] - anchor)),
        'cut_seam_gap_m': float(max(seam_errors)),
        'stalk_world_transform': warp.tolist(),
        'fruit_body_world_rigid_transform': (
            new_world_from_fruit @ np.linalg.inv(old_world_from_fruit)).tolist(),
        'real_greenhouse_geometry_verified': False,
    }
    if single_plant:
        single_plant_fixture(output, world, scene, plant_include, fruit_include,
                             world_from_plant, new_world_from_fruit, anchor,
                             rotation @ old_tangent)
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
    args = parser.parse_args()
    print(prepare(args.output, args.direction, args.elevation_deg,
                  args.azimuth_offset_deg, args.length_scale, args.single_plant))
