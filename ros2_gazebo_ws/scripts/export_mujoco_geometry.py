#!/usr/bin/env python3
"""Export CAD visual groups and assembly transforms from the standalone MJCF."""
from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET
import warnings

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

WORKSPACE = Path(__file__).resolve().parents[1]
SOURCE = WORKSPACE.parent / 'assets/fk_reference/tomato_picker_standalone/tomato_picker_standalone.xml'
TARGET = WORKSPACE / 'src/tomato_sim_description'
ROOT = ET.parse(SOURCE).getroot()
ASSETS = {m.get('name'): m for m in ROOT.findall('asset/mesh')}
OUTPUT = ET.Element('robot', {'xmlns:xacro': 'http://www.ros.org/wiki/xacro'})


def vector(element, key, default):
    return np.fromstring(element.get(key, default), sep=' ')


def values(vector):
    return ' '.join(format(v, '.11g') for v in vector)


def rotation(element):
    if 'quat' in element.attrib:
        w, x, y, z = vector(element, 'quat', '1 0 0 0')
        return Rotation.from_quat([x, y, z, w])
    return Rotation.from_euler('xyz', vector(element, 'euler', '0 0 0'))


def body(name):
    return ROOT.find(f".//body[@name='{name}']")


def macro(name):
    return ET.SubElement(OUTPUT, 'xacro:macro', name=name, params='')


def origin(parent, xyz, orient):
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', message='Gimbal lock detected')
        ET.SubElement(parent, 'origin', xyz=values(xyz), rpy=values(orient.as_euler('xyz')))


def inertial(parent, source):
    node = ET.SubElement(parent, 'inertial')
    origin(node, vector(source, 'pos', '0 0 0'), rotation(source))
    ET.SubElement(node, 'mass', value=source.get('mass'))
    if 'fullinertia' in source.attrib:
        entries = vector(source, 'fullinertia', '')
    else:
        entries = [*vector(source, 'diaginertia', ''), 0, 0, 0]
    ET.SubElement(node, 'inertia', dict(zip(
        ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'), values(entries).split())))


def export_link(name, sources, mesh_filter=lambda geom: True, offset=None, source_inertia=True):
    dest = macro(name + '_cad')
    if source_inertia:
        inertial(dest, sources[0].find('inertial'))
    groups = defaultdict(list)
    for source in sources:
        for geom in source.findall('geom'):
            if geom.get('group', '1') != '1' or not mesh_filter(geom):
                continue
            if geom.get('type') == 'cylinder':
                prim = ET.SubElement(dest, 'visual', name=geom.get('name'))
                origin(prim, vector(geom, 'pos', '0 0 0'), rotation(geom))
                radius, half_length = vector(geom, 'size', '')
                ET.SubElement(ET.SubElement(prim, 'geometry'), 'cylinder',
                              radius=str(radius), length=str(2*half_length))
                ET.SubElement(ET.SubElement(prim, 'material', name=geom.get('name')),
                              'color', rgba=geom.get('rgba'))
                continue
            asset = ASSETS[geom.get('mesh')]
            mesh = trimesh.load_mesh(SOURCE.parent / asset.get('file'), process=False)
            mesh.vertices *= vector(asset, 'scale', '1 1 1')
            mesh.vertices = rotation(geom).apply(mesh.vertices) + vector(geom, 'pos', '0 0 0')
            if offset is not None:
                mesh.vertices -= offset
            groups[geom.get('rgba', '0.55 0.57 0.60 1')].append(mesh)
    for index, (color, meshes) in enumerate(groups.items()):
        part = f'{name}_{index}'
        trimesh.util.concatenate(meshes).export(TARGET / f'meshes/{part}.stl')
        visual = ET.SubElement(dest, 'visual', name=part)
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                      filename=f'package://tomato_sim_description/meshes/{part}.stl')
        ET.SubElement(ET.SubElement(visual, 'material', name=part), 'color', rgba=color)
    return dest


chassis, lift, tool = (body(name) for name in ('chassis', 'lift', 'base_link'))
export_link('chassis', [chassis, *[b for b in chassis.findall('body')]])
# The crate CAD is mounted to the carriage at the zero-height baseline pose.
def is_crate(geom):
    return geom.get('name') == 'v_part_330'


export_link('lift', [lift], lambda g: not is_crate(g))
basket_pos = vector(lift.find("site[@name='basket_site']"), 'pos', '')
basket = export_link('basket', [lift], is_crate, offset=basket_pos, source_inertia=False)
for geom in lift.findall('geom'):
    if geom.get('name', '').startswith('crate_col_'):
        collision = ET.SubElement(basket, 'collision', name=geom.get('name'))
        origin(collision, vector(geom, 'pos', '') - basket_pos, rotation(geom))
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box',
                      size=values(vector(geom, 'size', '') * 2))

export_link('cutter', [tool])
for old, new in (('gear_link', 'gear'), ('left_finger_link', 'left_finger'),
                 ('right_finger_link', 'right_finger')):
    source = body(old)
    export_link(new, [source])
    origin(macro(new + '_origin'), vector(source, 'pos', '0 0 0'), rotation(source))

origin(macro('flange_origin'), vector(tool, 'pos', '0 0 0'), Rotation.identity())
origin(macro('cutter_origin'), [0, 0, 0], rotation(tool))
origin(macro('basket_origin'), basket_pos, Rotation.identity())
orient = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
camera = tool.find("camera[@name='cam_wrist_d405']")
origin(macro('wrist_camera_origin'), vector(camera, 'pos', '0 0 0'),
       rotation(camera) * Rotation.from_matrix(orient))

ET.indent(OUTPUT)
ET.ElementTree(OUTPUT).write(TARGET / 'urdf/mujoco_cad.xacro', encoding='utf-8',
                            xml_declaration=True)
print('Exported CAD and assembly macros:', len(list(OUTPUT)))
