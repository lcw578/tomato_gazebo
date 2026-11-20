#!/usr/bin/env python3
"""Derive a detachable target from v4 without relocating any map entity."""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import prepare_phase3_world as common

from simulation_config import harvest_world_path, read_world, sha256_file

CONFIG = common.CONFIG
ROOT = common.ROOT
MIGRATION = common.MIGRATION

LEGACY_PARAMETERS = {
    'tcp_reference': 'contact_tcp -> Gazebo tool_tcp',
    'approach_translation_tolerance_m': 0.005,
    'approach_rotation_tolerance_rad': 0.03,
    'run_main_stem_clearance_m': 0.003,
    'plan_main_stem_clearance_m': 0.005,
    'contact_report_cap_m': 0.008,
    'closed_blade_plane_x_m_in_cutter_base': -0.004897326065632642,
    'closed_blade_thickness_m': 0.00020000076641136892,
    'slot_y_m_in_cutter_base': [-0.0075, 0.0025],
    'slot_z_m_in_cutter_base': [-0.094, -0.052],
    'max_natural_crossing_angle_deg': 30.0,
    'cut_success_requires': [
        'actual_contact_with_exact_0.2mm_CAD_blade',
        'contact_with_split_peduncle_collision',
        'contact_point_inside_closed_slot',
        'TCP_within_5mm_of_ground_truth_cutpoint',
        'no_main_stem_contact',
    ],
}


def contact_only(collision):
    surface = collision.find('surface')
    if surface is None:
        surface = ET.SubElement(collision, 'surface')
    contact = surface.find('contact')
    if contact is None:
        contact = ET.SubElement(surface, 'contact')
    setting = contact.find('collide_without_contact')
    if setting is None:
        setting = ET.SubElement(contact, 'collide_without_contact')
    setting.text = 'true'


def set_fruit_sphere_inertia(link):
    """Distribute the existing payload mass over its solid fruit spheres."""
    spheres = []
    for collision in link.findall('collision'):
        radius = collision.findtext('geometry/sphere/radius')
        if radius is None or collision.get('name', '').startswith('contact_stalk_'):
            continue
        center, _ = common.parse_xyz_rpy(collision.findtext('pose'))
        spheres.append((center, float(radius)))
    if not spheres:
        raise ValueError('Payload fruit sphere collision geometry is missing')
    inertial = link.find('inertial')
    mass = float(inertial.findtext('mass'))
    total_volume_factor = sum(radius**3 for _, radius in spheres)
    sphere_masses = [mass * radius**3 / total_volume_factor for _, radius in spheres]
    center_of_mass = [sum(sphere_mass * center[axis]
                          for (center, _), sphere_mass in zip(spheres, sphere_masses)) / mass
                      for axis in range(3)]
    moments = dict.fromkeys(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'), 0.0)
    for (center, radius), sphere_mass in zip(spheres, sphere_masses):
        x, y, z = [center[axis] - center_of_mass[axis] for axis in range(3)]
        sphere_inertia = 0.4 * sphere_mass * radius**2
        moments['ixx'] += sphere_inertia + sphere_mass * (y*y + z*z)
        moments['iyy'] += sphere_inertia + sphere_mass * (x*x + z*z)
        moments['izz'] += sphere_inertia + sphere_mass * (x*x + y*y)
        moments['ixy'] -= sphere_mass * x*y
        moments['ixz'] -= sphere_mass * x*z
        moments['iyz'] -= sphere_mass * y*z
    pose = inertial.find('pose')
    if pose is None:
        pose = ET.SubElement(inertial, 'pose')
    pose.text = ' '.join(f'{value:.16g}' for value in (*center_of_mass, 0, 0, 0))
    inertia = inertial.find('inertia')
    if inertia is None:
        inertia = ET.SubElement(inertial, 'inertia')
    for name, value in moments.items():
        component = inertia.find(name)
        if component is None:
            component = ET.SubElement(inertia, name)
        component.text = f'{value:.16g}'


def add_stem_sensor(link, model_name, instance):
    collisions = [x.get('name') for x in link.findall('collision')
                  if '_stem' in x.get('name', '')]
    if not collisions:
        raise ValueError(f'Main-stem collision missing in {model_name}')
    sensor = ET.SubElement(link, 'sensor', {'name': f'{model_name}_stem_contact', 'type': 'contact'})
    ET.SubElement(sensor, 'update_rate').text = '100'
    contact = ET.SubElement(sensor, 'contact')
    for name in collisions:
        ET.SubElement(contact, 'collision').text = name
    plugin = ET.SubElement(sensor, 'plugin', {
        'name': f'{model_name}_stem_contact_ros', 'filename': 'libgazebo_ros_bumper.so'})
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/sim/harvest'
    ET.SubElement(ros, 'remapping').text = f'bumper_states:=main_stem_contacts/{instance}'
    ET.SubElement(plugin, 'frame_name').text = 'world'
    return len(collisions)


def save_instrumented_model(source, name, instance, output):
    tree = ET.parse(source)
    model = tree.getroot().find('model')
    model.set('name', name)
    plant = next(x for x in model.findall('link') if x.get('name', '').startswith('plant_'))
    add_stem_sensor(plant, name, instance)
    folder = output / 'models' / name
    folder.mkdir(parents=True, exist_ok=True)
    tree.write(folder / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (folder / 'model.config').write_text(
        f'<model><name>{name}</name><version>1.0</version>'
        '<sdf version="1.7">model.sdf</sdf></model>\n')


def map_point(point, old_pose, new_pose):
    old_xyz, old_rpy = common.parse_xyz_rpy(old_pose)
    new_xyz, new_rpy = common.parse_xyz_rpy(new_pose)
    local = common.inverse_rotate_rpy([point[i] - old_xyz[i] for i in range(3)], old_rpy)
    rotated = common.rotate_rpy(local, new_rpy)
    return [new_xyz[i] + rotated[i] for i in range(3)]


def prepare(world_path=None, include_name=None, output=None):
    world_path = world_path or CONFIG['world']
    include_name = include_name or CONFIG['candidate_harvest_station']['plant_instance']
    world_path = Path(world_path).resolve()
    output = Path(output) if output is not None else ROOT / 'validation/phase3'
    common.OUT = output
    tree, world = read_world(world_path)
    if world.get('name') != CONFIG['baseline_id']:
        raise ValueError('Harvest baseline must be the user-selected tomato_corridor_v4 world')
    original = {x.findtext('name'): deepcopy(x) for x in world.findall('include')}
    target_include = next((x for x in world.findall('include')
                           if x.findtext('name') == include_name), None)
    if target_include is None:
        raise ValueError(f'Target instance {include_name} not present in v4')
    source_name = target_include.findtext('uri').removeprefix('model://')
    if not source_name.startswith('corridor_v4_plant_'):
        raise ValueError(f'Unsupported v4 target template {source_name}')
    full_source_name = source_name.replace('corridor_v4_plant_', 'corridor_plant_', 1)
    models = MIGRATION / 'assets/models'
    sdf = ET.parse(models / source_name / 'model.sdf').getroot().find('model')
    full = ET.parse(models / full_source_name / 'model.sdf').getroot().find('model')
    links = {x.get('name'): x for x in sdf.findall('link')}
    if 'truss_9_1' not in links:
        raise ValueError('This audited target is plant 9, truss 1; choose a matching v4 instance')
    plant = links['plant_9']
    fruit = links['truss_9_1']
    full_fruit = full.find("link[@name='truss_9_1']")
    fruit_pose = fruit.findtext('pose') or '0 0 0 0 0 0'
    plant_pose = target_include.findtext('pose') or '0 0 0 0 0 0'
    v2 = ET.parse(MIGRATION / 'tomato_corridor_v2.world').getroot().find('world')
    old_include = next(x for x in v2.findall('include')
                       if x.findtext('name') == full_source_name)
    old_pose = old_include.findtext('pose')
    truth = json.loads((MIGRATION / 'truss_ground_truth.json').read_text())
    row = next(x for x in truth['trusses'] if x['instance'] == full_source_name
               and x['plant'] == 9 and x['truss_index'] == 1)
    cut = map_point(row['cut_center_world_xyz_m'], old_pose, plant_pose)
    old_rpy = common.parse_xyz_rpy(old_pose)[1]
    new_rpy = common.parse_xyz_rpy(plant_pose)[1]
    tangent = common.rotate_rpy(common.inverse_rotate_rpy(
        row['peduncle_axis_world_unit'], old_rpy), new_rpy)
    # v4 keeps stalk visuals but drops their fine collision proxies. Restore
    # only the selected target's original proxies, with no visual replacement.
    restored = 0
    for collision in full_fruit.findall('collision'):
        if collision.get('name', '').startswith('contact_stalk_'):
            fruit.append(deepcopy(collision))
            restored += 1
    if restored == 0:
        raise ValueError('The original target peduncle collision chain is missing')
    # Re-express the complete stalk in the mother-plant frame, then reuse
    # the audited cut-plane split. Its upper half returns to the truss.
    for tag in ('visual', 'collision'):
        for node in list(fruit.findall(tag)):
            if node.get('name', '').startswith('contact_stalk_'):
                fruit.remove(node)
                node.find('pose').text = common.compose_pose(fruit_pose, node.findtext('pose'))
                plant.append(node)
    split = common.split_stalk(plant, fruit, plant_pose, fruit_pose, cut, tangent)
    if not split['split_visual_segment']:
        raise ValueError('The cut plane did not intersect the target stalk')
    # The stalk still reports exact ODE contact points while the blade cuts
    # through it. A kinematic payload must also avoid pushing the robot while
    # SetEntityState carries it. Dynamic release restores fruit-body contact.
    for collision in plant.findall('collision'):
        if collision.get('name', '').startswith('contact_stalk_'):
            contact_only(collision)
    for collision in fruit.findall('collision'):
        contact_only(collision)
    set_fruit_sphere_inertia(fruit)
    stem_names = [x.get('name') for x in plant.findall('collision')
                  if '_stem' in x.get('name', '')]
    common.write_model('phase3_plant_9', [plant], movable=False, contact_collisions=stem_names,
                       contact_topic=f'main_stem_contacts/{include_name}')
    target_include.find('uri').text = 'model://phase3_plant_9'
    for index in range(4):
        link = links[f'truss_9_{index}']
        model_name = f'phase3_fruit_truss_9_{index}'
        common.write_model(model_name, [link], movable=True,
                           contact_collisions=[x.get('name') for x in link.findall('collision')],
                           contact_sensor_name='payload_contact', contact_topic='payload_contacts')
        item = ET.SubElement(world, 'include')
        ET.SubElement(item, 'uri').text = f'model://{model_name}'
        ET.SubElement(item, 'name').text = f'fruit_truss_9_{index}'
        ET.SubElement(item, 'pose').text = common.compose_pose(plant_pose, link.findtext('pose'))
    instrumented = {}
    for item in world.findall('include'):
        if item is target_include:
            continue
        uri = item.findtext('uri') or ''
        if not uri.startswith('model://corridor_v4_plant_'):
            continue
        template = uri.removeprefix('model://')
        # Repeated includes of one template still need unique ROS plugin names.
        instance = item.findtext('name')
        name = f'phase3_v4_{instance}'
        save_instrumented_model(models / template / 'model.sdf', name, instance, output)
        instrumented[instance] = name
        item.find('uri').text = f'model://{name}'
    current = {x.findtext('name'): x for x in world.findall('include')}
    for name, before in original.items():
        if current[name].findtext('pose') != before.findtext('pose'):
            raise ValueError(f'v4 map pose changed unexpectedly: {name}')
        if not name.startswith('plant_') and ET.tostring(current[name]) != ET.tostring(before):
            raise ValueError(f'v4 fixture changed unexpectedly: {name}')
    plugin = ET.SubElement(world, 'plugin', {
        'name': 'gazebo_ros_state', 'filename': 'libgazebo_ros_state.so'})
    ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/'
    ET.SubElement(plugin, 'update_rate').text = '50'
    output.mkdir(parents=True, exist_ok=True)
    out_world = harvest_world_path(CONFIG, output)
    tree.write(out_world, encoding='utf-8', xml_declaration=True)
    # A real centerline point is used only by the deliberate stem-contact test.
    full_plant = full.find("link[@name='plant_9']")
    stem_centers = [common.parse_xyz_rpy(x.findtext('pose'))[0]
                    for x in full_plant.findall('collision')
                    if '_stem' in x.get('name', '') and x.find('geometry/cylinder') is not None]
    attachment_local = common.inverse_rotate_rpy(
        [row['A_world_xyz_m'][i] - common.parse_xyz_rpy(old_pose)[0][i] for i in range(3)], old_rpy)
    if not stem_centers:
        raise ValueError('Original main-stem centerline proxies are missing')
    probe_local = min(stem_centers, key=lambda p: sum((p[i] - attachment_local[i])**2 for i in range(3)))
    root_xyz = common.parse_xyz_rpy(plant_pose)[0]
    rotated_probe = common.rotate_rpy(probe_local, new_rpy)
    summary = {
        'baseline_id': CONFIG['baseline_id'], 'source_world': str(world_path),
        'source_world_sha256': sha256_file(world_path),
        'world': str(out_world), 'plant_model': include_name,
        'target_template': source_name, 'truth_source_instance': full_source_name,
        'target_model': 'fruit_truss_9_1',
        'release_sdf': 'models/phase3_fruit_truss_9_1/model.sdf',
        'plant_include_pose': plant_pose,
        'station_alignment': {'applied_y_shift_m': 0.0, 'reference': 'v4 source pose preserved'},
        'fruit_local_pose': fruit_pose, 'cut_point_world': cut,
        'cut_point_truss_local': common.relative_position(cut, plant_pose, fruit_pose),
        'peduncle_tangent_world': tangent,
        'main_stem_probe_world': [root_xyz[i] + rotated_probe[i] for i in range(3)],
        'main_stem_tangent_world': common.rotate_rpy(common.inverse_rotate_rpy(
            row['stem_tangent_unit'], old_rpy), new_rpy),
        'main_stem_contact_topics': [f'/sim/harvest/main_stem_contacts/{name}'
                                     for name in original if name.startswith('plant_')],
        'stalk_split': split,
        'v4_integrity': {'original_entities_preserved': len(original),
                         'plant_instances_preserved': sum(n.startswith('plant_') for n in original),
                         'all_original_include_poses_unchanged': True,
                         'fixtures_and_track_unchanged': True,
                         'detached_top_level_trusses': 4,
                         'main_stem_sensor_instances': 60},
        'collision_changes': {'restored_target_stalk_proxies': restored,
                              'removed_v4_collisions': 0,
                              'non_target_visuals_modified': False},
        'legacy_parameters': LEGACY_PARAMETERS,
    }
    (output / 'phase3_scene.json').write_text(json.dumps(summary, indent=2) + '\n')
    return out_world


if __name__ == '__main__':
    print(prepare())
