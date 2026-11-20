#!/usr/bin/env python3
"""Create a workspace-local Gazebo world with independently movable fruit trusses.

The command entrypoint uses the user-selected v4 map. Geometry helpers and an
explicitly named legacy builder remain for historical code comparison only.
Generated worlds and SDF models live below validation/phase3.
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from simulation_config import load_configuration, workspace

ROOT = workspace()
CONFIG = load_configuration()
MIGRATION = Path(CONFIG['asset_root'])
DEFAULT_WORLD = Path(CONFIG['world'])
OUT = ROOT / 'validation' / 'phase3'


def parse_pose(text: str | None) -> tuple[list[float], list[float]]:
    values = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    if len(values) != 6:
        raise ValueError(f'Expected six pose values, got {text!r}')
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = [cr * cp * cy + sr * sp * sy,
         sr * cp * cy - cr * sp * sy,
         cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy]
    return [x, y, z], q


def compose_pose(parent_text: str | None, child_text: str | None) -> str:
    p, q = parse_pose(parent_text)
    c, r = parse_pose(child_text)
    w, x, y, z = q
    cx, cy, cz = c
    rotated = [cx * (1 - 2 * (y*y + z*z)) + cy * 2 * (x*y - z*w) + cz * 2 * (x*z + y*w),
               cx * 2 * (x*y + z*w) + cy * (1 - 2 * (x*x + z*z)) + cz * 2 * (y*z - x*w),
               cx * 2 * (x*z - y*w) + cy * 2 * (y*z + x*w) + cz * (1 - 2 * (x*x + y*y))]
    pos = [a + b for a, b in zip(p, rotated)]
    rw, rx, ry, rz = r
    out = [w*rw - x*rx - y*ry - z*rz,
           w*rx + x*rw + y*rz - z*ry,
           w*ry - x*rz + y*rw + z*rx,
           w*rz + x*ry - y*rx + z*rw]
    # SDF model include poses are roll-pitch-yaw; convert the composed quaternion.
    qw, qx, qy, qz = out
    roll = math.atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy))
    sinp = 2 * (qw*qy - qz*qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
    return ' '.join(f'{v:.12g}' for v in (*pos, roll, pitch, yaw))


def relative_position(world_point: list[float], parent_text: str | None,
                      child_text: str | None) -> list[float]:
    parent, parent_rpy = parse_xyz_rpy(parent_text)
    child, child_rpy = parse_xyz_rpy(child_text)
    in_parent = inverse_rotate_rpy([world_point[i] - parent[i] for i in range(3)], parent_rpy)
    return inverse_rotate_rpy([in_parent[i] - child[i] for i in range(3)], child_rpy)


def parse_xyz_rpy(text: str | None) -> tuple[list[float], list[float]]:
    values = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    if len(values) != 6:
        raise ValueError(f'Expected six pose values, got {text!r}')
    return values[:3], values[3:]


def rotate_rpy(vector: list[float], rpy: list[float]) -> list[float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
              [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
              [-sp, cp*sr, cp*cr]]
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def inverse_rotate_rpy(vector: list[float], rpy: list[float]) -> list[float]:
    columns = [rotate_rpy(axis, rpy) for axis in ([1.0, 0.0, 0.0],
                                                   [0.0, 1.0, 0.0],
                                                   [0.0, 0.0, 1.0])]
    return [sum(columns[col][row] * vector[row] for row in range(3))
            for col in range(3)]


def rpy_from_z_axis(axis: list[float]) -> list[float]:
    x, y, z = axis
    norm = math.sqrt(x*x + y*y + z*z)
    x, y, z = x/norm, y/norm, z/norm
    yaw = math.atan2(y, x)
    # Rz(yaw) * Ry(pitch) maps the local +Z axis to (x,y,z).
    pitch = math.atan2(math.hypot(x, y), z)
    return [0.0, pitch, yaw]


def split_stalk(plant: ET.Element, fruit: ET.Element, plant_pose: str,
                fruit_local_pose: str, cut_world: list[float], tangent_world: list[float]) -> dict:
    """Split the legacy 9_1 centerline tubes at its reference cut plane."""
    plant_xyz, plant_rpy = parse_xyz_rpy(plant_pose)
    fruit_xyz, fruit_rpy = parse_xyz_rpy(fruit_local_pose)
    cut_local_plant = inverse_rotate_rpy([cut_world[i] - plant_xyz[i] for i in range(3)],
                                         plant_rpy)
    tangent_local = inverse_rotate_rpy(tangent_world, plant_rpy)
    transferred = {'visuals': 0, 'collisions': 0, 'split_visual_segment': None}

    def local_point(point: list[float]) -> list[float]:
        return inverse_rotate_rpy([point[i] - fruit_xyz[i] for i in range(3)], fruit_rpy)

    def pose_geometry(node: ET.Element) -> tuple[list[float], list[float], float, float]:
        xyz, angles = parse_xyz_rpy(node.findtext('pose'))
        shape = node.find('geometry/cylinder')
        if shape is None:
            raise ValueError(f'Expected centerline cylinder at {node.get("name")}')
        return xyz, angles, float(shape.findtext('length')), float(shape.findtext('radius'))

    def create_half(node: ET.Element, start: list[float], end: list[float],
                    radius: float, fruit_side: bool) -> None:
        if math.dist(start, end) <= 1e-8:
            return
        clone = ET.fromstring(ET.tostring(node, encoding='unicode'))
        if fruit_side:
            xyz = local_point([(start[i]+end[i])/2 for i in range(3)])
            direction_world = [end[i]-start[i] for i in range(3)]
            direction_local = inverse_rotate_rpy(direction_world, fruit_rpy)
            length = math.dist(start, end)
        else:
            xyz = [(start[i]+end[i])/2 for i in range(3)]
            direction_local = [end[i]-start[i] for i in range(3)]
            length = math.dist(start, end)
        angles = rpy_from_z_axis(direction_local)
        clone.find('pose').text = ' '.join(f'{v:.12g}' for v in (*xyz, *angles))
        clone.find('geometry/cylinder/length').text = f'{length:.12g}'
        clone.find('geometry/cylinder/radius').text = f'{radius:.12g}'
        (fruit if fruit_side else plant).append(clone)
        if fruit_side:
            transferred['visuals' if clone.tag == 'visual' else 'collisions'] += 1

    # Segment visuals and their hidden collision cylinders are transferred by
    # centerline side.  A straddling segment is shortened to the same plane in
    # both links, preserving the paired ownership at the cut.
    for tag in ('visual', 'collision'):
        for node in list(plant.findall(tag)):
            name = node.get('name', '')
            if not name.startswith(('contact_stalk_visual_plant_', 'contact_stalk_plant_')):
                continue
            if '_cap_' in name:
                center, _ = parse_xyz_rpy(node.findtext('pose'))
                if sum((center[i]-cut_local_plant[i])*tangent_local[i] for i in range(3)) > 0:
                    clone = ET.fromstring(ET.tostring(node, encoding='unicode'))
                    xyz = local_point(center)
                    angles = parse_xyz_rpy(node.findtext('pose'))[1]
                    clone.find('pose').text = ' '.join(f'{v:.12g}' for v in (*xyz, *angles))
                    fruit.append(clone)
                    transferred['collisions'] += int(tag == 'collision')
                    plant.remove(node)
                continue
            xyz, angles, length, radius = pose_geometry(node)
            z_axis = rotate_rpy([0.0, 0.0, 1.0], angles)
            start = [xyz[i] - z_axis[i]*length/2 for i in range(3)]
            end = [xyz[i] + z_axis[i]*length/2 for i in range(3)]
            s0 = sum((start[i]-cut_local_plant[i])*tangent_local[i] for i in range(3))
            s1 = sum((end[i]-cut_local_plant[i])*tangent_local[i] for i in range(3))
            if s0 > 0 and s1 > 0:
                clone = ET.fromstring(ET.tostring(node, encoding='unicode'))
                xyz_fruit = local_point(xyz)
                direction = inverse_rotate_rpy(z_axis, fruit_rpy)
                clone.find('pose').text = ' '.join(f'{v:.12g}' for v in (*xyz_fruit, *rpy_from_z_axis(direction)))
                fruit.append(clone)
                transferred['visuals' if tag == 'visual' else 'collisions'] += 1
                plant.remove(node)
            elif s0*s1 < 0:
                t = -s0 / (s1-s0)
                crossing = [start[i] + (end[i]-start[i])*t for i in range(3)]
                if s0 < 0:
                    create_half(node, start, crossing, radius, False)
                    create_half(node, crossing, end, radius, True)
                else:
                    create_half(node, end, crossing, radius, False)
                    create_half(node, crossing, start, radius, True)
                transferred['split_visual_segment'] = name
                plant.remove(node)

    # Joint spheres at the boundary do not cross ownership: MuJoCo removes
    # them when their radius would extend past the cut plane.
    for tag in ('visual', 'collision'):
        for node in list(plant.findall(tag)):
            if 'contact_stalk_joint_plant_' not in node.get('name', ''):
                continue
            xyz, _ = parse_xyz_rpy(node.findtext('pose'))
            radius = float(node.findtext('geometry/sphere/radius'))
            signed = sum((xyz[i]-cut_local_plant[i])*tangent_local[i] for i in range(3))
            if signed - radius > 0:
                clone = ET.fromstring(ET.tostring(node, encoding='unicode'))
                clone.find('pose').text = ' '.join(f'{v:.12g}' for v in (*local_point(xyz), 0, 0, 0))
                fruit.append(clone)
                transferred['visuals' if tag == 'visual' else 'collisions'] += 1
                plant.remove(node)
            elif signed + radius > 0:
                plant.remove(node)
    return transferred


def write_model(name: str, links: list[ET.Element], *, movable: bool,
                contact_collisions: list[str] | None = None,
                contact_sensor_name: str = 'main_stem_contact',
                contact_topic: str = 'main_stem_contacts') -> Path:
    sdf = ET.Element('sdf', {'version': '1.7'})
    model = ET.SubElement(sdf, 'model', {'name': name})
    ET.SubElement(model, 'static').text = 'false' if movable else 'true'
    for source_link in links:
        link = ET.fromstring(ET.tostring(source_link, encoding='unicode'))
        # Each extracted link becomes the independent model's root link.
        link.attrib['name'] = 'plant' if not movable else 'fruit'
        link.attrib.pop('pose', None)
        link.find('pose') is not None and link.remove(link.find('pose'))
        if movable:
            grav = link.find('gravity')
            if grav is None:
                grav = ET.SubElement(link, 'gravity')
            grav.text = 'false'
            kin = link.find('kinematic')
            if kin is None:
                kin = ET.SubElement(link, 'kinematic')
            kin.text = 'true'
        if contact_collisions:
            sensor = ET.SubElement(link, 'sensor', {
                'name': f'{name}_{contact_sensor_name}', 'type': 'contact'})
            ET.SubElement(sensor, 'update_rate').text = '100'
            contact_node = ET.SubElement(sensor, 'contact')
            for collision in contact_collisions:
                ET.SubElement(contact_node, 'collision').text = collision
            plugin = ET.SubElement(sensor, 'plugin', {
                'name': f'{name}_{contact_sensor_name}_ros',
                'filename': 'libgazebo_ros_bumper.so'})
            ros = ET.SubElement(plugin, 'ros')
            ET.SubElement(ros, 'namespace').text = '/sim/harvest'
            ET.SubElement(ros, 'remapping').text = f'bumper_states:={contact_topic}'
            ET.SubElement(plugin, 'frame_name').text = 'world'
        model.append(link)
    folder = OUT / 'models' / name
    folder.mkdir(parents=True, exist_ok=True)
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(folder / 'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.Element('model', {'name': name})
    ET.SubElement(config, 'name').text = name
    ET.SubElement(config, 'version').text = '1.0'
    ET.SubElement(config, 'sdf', {'version': '1.7'}).text = 'model.sdf'
    ET.indent(config, space='  ')
    ET.ElementTree(config).write(folder / 'model.config', encoding='utf-8', xml_declaration=True)
    return folder


def prepare_legacy_single_plant(world_path: Path, include_name: str = 'verify_plant_9',
                               station_y: float = -0.475) -> Path:
    source_models = MIGRATION / 'assets' / 'models'
    world = ET.parse(world_path)
    root = world.getroot()
    include = next((node for node in root.findall('.//include')
                    if node.findtext('name') == include_name), None)
    if include is None:
        raise ValueError(f'No include named {include_name!r} in {world_path}')
    world_node = root.find('.//world')
    assert world_node is not None
    plant_xyz, plant_rpy = parse_xyz_rpy(include.findtext('pose'))
    source_plant_y = plant_xyz[1]
    station_shift_y = station_y - source_plant_y
    for item in world_node.findall('include'):
        if item.findtext('name') not in ('verify_bed', 'verify_pot', include_name):
            continue
        xyz, rpy = parse_xyz_rpy(item.findtext('pose'))
        xyz[1] += station_shift_y
        if item.findtext('name') == 'verify_bed':
            # Keep the bed under the pot while clearing the 0.79 m chassis shell.
            # Centering the bed on the shifted plant overlaps the shell at spawn.
            xyz[1] -= 0.10
        item.find('pose').text = ' '.join(f'{v:.12g}' for v in (*xyz, *rpy))
    uri = include.findtext('uri')
    if not uri or not uri.startswith('model://'):
        raise ValueError(f'Unsupported plant URI: {uri!r}')
    source_name = uri.removeprefix('model://')
    source = source_models / source_name / 'model.sdf'
    if not source.is_file():
        raise FileNotFoundError(source)
    sdf = ET.parse(source).getroot().find('model')
    links = {link.get('name'): link for link in sdf.findall('link')}
    plant_link = next((link for name, link in links.items() if name.startswith('plant_')), None)
    if plant_link is None:
        raise ValueError(f'No plant root link in {source}')

    OUT.mkdir(parents=True, exist_ok=True)
    # Extract the selected fruit link before removing its root pose.
    truss_link = links['truss_9_1']
    plant_pose = include.findtext('pose') or '0 0 0 0 0 0'
    fruit_local_pose = truss_link.findtext('pose') or '0 0 0 0 0 0'
    truth = __import__('json').loads((MIGRATION / 'truss_ground_truth.json').read_text())
    truth_row = next(row for row in truth['trusses']
                     if row['instance'] == 'corridor_plant_s0_i7'
                     and row['plant'] == 9 and row['truss_index'] == 1)
    ref_root = [1.75, -1.0, 0.249153]
    cut_world = [include_xyz + truth_xyz - ref_xyz
                 for include_xyz, truth_xyz, ref_xyz in zip(
                     parse_xyz_rpy(plant_pose)[0], truth_row['cut_center_world_xyz_m'], ref_root)]
    tangent_world = truth_row['peduncle_axis_world_unit']
    stalk_split = split_stalk(plant_link, truss_link, plant_pose, fruit_local_pose,
                              cut_world, tangent_world)
    target_cut_stalk = stalk_split['split_visual_segment']
    removed_collisions = 0
    for collision in list(plant_link.findall('collision')):
        name = collision.get('name', '')
        if name.startswith('g_v9_stem') or name == target_cut_stalk \
                or name.startswith(target_cut_stalk + '_'):
            continue
        plant_link.remove(collision)
        removed_collisions += 1
    contacts = [node.get('name') for node in plant_link.findall('collision')
                if node.get('name', '').startswith('g_v9_stem')]
    if not contacts:
        raise ValueError('No main-stem collision geometry in the plant SDF')
    write_model('phase3_plant_9', [plant_link], movable=False,
                contact_collisions=contacts)
    include.find('uri').text = 'model://phase3_plant_9'
    # Remove the link names from the generated root so each truss is an entity.
    # ElementTree does not retain parents; add fruit includes to the same world.
    if not any(plugin.get('name') == 'gazebo_ros_state'
               for plugin in world_node.findall('plugin')):
        plugin = ET.SubElement(world_node, 'plugin', {
            'name': 'gazebo_ros_state', 'filename': 'libgazebo_ros_state.so'})
        ros = ET.SubElement(plugin, 'ros')
        ET.SubElement(ros, 'namespace').text = '/'
        ET.SubElement(plugin, 'update_rate').text = '50'
    for joint in sdf.findall('joint'):
        child = joint.findtext('child')
        if not child or not child.startswith('truss_9_'):
            continue
        index = int(child.rsplit('_', 1)[1])
        fruit_link = links[child]
        if index == 1:
            fruit_link = truss_link
        model_name = f'phase3_fruit_truss_9_{index}'
        fruit_collisions = [node.get('name') for node in fruit_link.findall('collision')
                            if node.get('name')]
        write_model(model_name, [fruit_link], movable=True,
                    contact_collisions=fruit_collisions,
                    contact_sensor_name='payload_contact', contact_topic='payload_contacts')
        model_pose = compose_pose(plant_pose, fruit_link.findtext('pose'))
        fruit = ET.SubElement(world_node, 'include')
        ET.SubElement(fruit, 'uri').text = f'model://{model_name}'
        ET.SubElement(fruit, 'name').text = f'fruit_truss_9_{index}'
        ET.SubElement(fruit, 'pose').text = model_pose
    # Emit the transformed test world without changing the migration source.
    out_world = OUT / f'{world_path.stem}_phase3.world'
    ET.indent(root, space='  ')
    world.write(out_world, encoding='utf-8', xml_declaration=True)
    (OUT / 'phase3_scene.json').write_text(__import__('json').dumps({
        'world': str(out_world), 'plant_model': 'verify_plant_9',
        'target_model': 'fruit_truss_9_1', 'plant_include_pose': plant_pose,
        'station_alignment': {
            'source_plant_y_m': source_plant_y,
            'verification_plant_y_m': station_y,
            'applied_y_shift_m': station_shift_y,
            'reference': 'MuJoCo F0-F2 fixed-station cutpoint Y ~= -0.438 m; plant root Y ~= -0.475 m',
        },
        'fruit_local_pose': fruit_local_pose, 'cut_point_world': cut_world,
        'cut_point_truss_local': relative_position(cut_world, plant_pose, fruit_local_pose),
        'peduncle_tangent_world': tangent_world,
        'stalk_split': stalk_split,
        'collision_simplification': {
            'removed_non_target_parent_collisions': removed_collisions,
            'retained_main_stem_prefix': 'g_v9_stem',
            'retained_target_cut_stalk': target_cut_stalk,
            'visuals_modified': False,
        },
        'legacy_parameters': {
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
        },
    }, indent=2) + '\n')
    return out_world


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=Path, default=DEFAULT_WORLD)
    parser.add_argument('--include-name', default=CONFIG['candidate_harvest_station']['plant_instance'])
    parser.add_argument('--station-y', type=float, default=None)
    args = parser.parse_args()
    if args.world.resolve() != DEFAULT_WORLD.resolve():
        raise ValueError(f'Harvest preparation requires the configured baseline {DEFAULT_WORLD}')
    if args.station_y is not None:
        raise ValueError('Do not relocate plant rows in the v4 baseline')
    from prepare_v4_harvest_world import prepare as prepare_v4
    print(prepare_v4(args.world, args.include_name))


if __name__ == '__main__':
    main()
