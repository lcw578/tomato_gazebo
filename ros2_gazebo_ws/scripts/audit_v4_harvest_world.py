#!/usr/bin/env python3
"""Independently compare generated harvest geometry against the user's v4 map."""
from collections import defaultdict
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from simulation_config import load_configuration, workspace

ROOT = workspace()
MIGRATION = Path(load_configuration()['asset_root'])
OUT = ROOT / 'validation/phase3'


def canonical(node, skip_sensors=False):
    attrs = dict(node.attrib)
    if node.tag == 'model':
        attrs.pop('name', None)
    return (node.tag, tuple(sorted(attrs.items())), (node.text or '').strip(),
            tuple(canonical(x, skip_sensors) for x in node
                  if not (skip_sensors and x.tag == 'sensor')))


def matrix(pose):
    values = list(map(float, (pose or '0 0 0 0 0 0').split()))
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def shapes(model, model_pose, kind):
    result = defaultdict(list)
    for link in model.findall('link'):
        parent = matrix(model_pose) @ matrix(model.findtext('pose')) @ matrix(link.findtext('pose'))
        for node in link.findall(kind):
            result[node.get('name')].append((node, parent @ matrix(node.findtext('pose'))))
    return result


def compare_shape(name, original, copies):
    source, transform = original
    shape = source.find('geometry')[0]
    kind = shape.tag
    if kind != 'cylinder' or len(copies) == 1:
        if len(copies) != 1:
            raise AssertionError(f'{name}: expected one {kind}, got {len(copies)}')
        node, current = copies[0]
        if kind == 'cylinder':
            current_shape = node.find('geometry/cylinder')
            assert current_shape is not None, name
            assert abs(float(current_shape.findtext('length')) - float(shape.findtext('length'))) < 1e-8, name
            assert abs(float(current_shape.findtext('radius')) - float(shape.findtext('radius'))) < 1e-10, name
            # A cylinder is unchanged by an axial reversal.
            assert abs(abs(np.dot(current[:3, 2], transform[:3, 2])) - 1) < 1e-8, name
        else:
            assert canonical(node.find('geometry')) == canonical(source.find('geometry')), name
            assert np.max(abs(current[:3, :3] - transform[:3, :3])) < 1e-8, name
        assert np.max(abs(current[:3, 3] - transform[:3, 3])) < 1e-8, name
        return
    assert len(copies) == 2, (name, len(copies))
    length = float(shape.findtext('length'))
    radius = float(shape.findtext('radius'))
    axis = transform[:3, 2]
    intervals = []
    for node, current in copies:
        cylinder = node.find('geometry/cylinder')
        assert cylinder is not None and abs(float(cylinder.findtext('radius')) - radius) < 1e-10, name
        delta = current[:3, 3] - transform[:3, 3]
        projected = float(delta @ axis)
        assert np.linalg.norm(delta - projected * axis) < 1e-8, name
        assert abs(abs(np.dot(current[:3, 2], axis)) - 1) < 1e-8, name
        half = float(cylinder.findtext('length')) / 2
        intervals.append((projected - half, projected + half))
    intervals.sort()
    assert abs(intervals[0][0] + length / 2) < 1e-8, (name, intervals, length)
    assert abs(intervals[-1][1] - length / 2) < 1e-8, name
    assert abs(intervals[0][1] - intervals[1][0]) < 1e-8, name


def main():
    scene = json.loads((OUT / 'phase3_scene.json').read_text())
    source_path = Path(scene['source_world'])
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == scene['source_world_sha256']
    source = ET.parse(source_path).getroot().find('world')
    generated = ET.parse(scene['world']).getroot().find('world')
    originals = {x.findtext('name'): x for x in source.findall('include')}
    current = {x.findtext('name'): x for x in generated.findall('include')}
    assert len(originals) == 94 and len(current) == 98
    selected = scene['plant_model']
    other_models_checked = set()
    plugins = set()
    for name, item in originals.items():
        assert name in current and item.findtext('pose') == current[name].findtext('pose'), name
        if not name.startswith('plant_'):
            assert canonical(item) == canonical(current[name]), name
        elif name != selected:
            old_uri = item.findtext('uri').removeprefix('model://')
            new_uri = current[name].findtext('uri').removeprefix('model://')
            old_model = ET.parse(MIGRATION / 'assets/models' / old_uri / 'model.sdf').getroot().find('model')
            new_model = ET.parse(OUT / 'models' / new_uri / 'model.sdf').getroot().find('model')
            assert canonical(old_model, True) == canonical(new_model, True), name
            plugin = new_model.find('.//sensor/plugin')
            assert plugin is not None and plugin.get('name') not in plugins, name
            plugins.add(plugin.get('name'))
            other_models_checked.add(name)
    old_template = originals[selected].findtext('uri').removeprefix('model://')
    old_model = ET.parse(MIGRATION / 'assets/models' / old_template / 'model.sdf').getroot().find('model')
    selected_models = [selected] + [f'fruit_truss_9_{i}' for i in range(4)]
    counts = {}
    for kind in ('visual', 'collision'):
        before = shapes(old_model, originals[selected].findtext('pose'), kind)
        after = defaultdict(list)
        for name in selected_models:
            item = current[name]
            uri = item.findtext('uri').removeprefix('model://')
            model = ET.parse(OUT / 'models' / uri / 'model.sdf').getroot().find('model')
            for key, entries in shapes(model, item.findtext('pose'), kind).items():
                after[key].extend(entries)
        for name, entries in before.items():
            assert len(entries) == 1, name
            if name not in after:
                # The legacy cut-plane rule removes a joint sphere whose
                # radius straddles the plane; do not permit any other loss.
                node, pose = entries[0]
                radius = node.findtext('geometry/sphere/radius')
                assert name.startswith('contact_stalk_joint_plant_') and radius, name
                signed = np.dot(pose[:3, 3] - scene['cut_point_world'],
                                scene['peduncle_tangent_world'])
                assert abs(signed) <= float(radius) + 1e-10, name
                continue
            compare_shape(name, entries[0], after[name])
        counts[kind] = {'original_shapes_checked': len(before),
                        'split_names': [name for name in before if len(after.get(name, [])) > 1],
                        'legacy_cut_boundary_joint_spheres_removed': [name for name in before
                                                                      if not after.get(name)]}
    # Derive the cut again from the serialized local deformation controls.
    truth = json.loads((MIGRATION / 'truss_ground_truth.json').read_text())
    row = next(x for x in truth['trusses'] if x['instance'] == scene['truth_source_instance']
               and x['truss_index'] == 1)
    v2 = ET.parse(MIGRATION / 'tomato_corridor_v2.world').getroot().find('world')
    old_root = next(x for x in v2.findall('include')
                    if x.findtext('name') == scene['truth_source_instance'])
    local = np.linalg.inv(matrix(old_root.findtext('pose'))) @ np.r_[row['cut_center_world_xyz_m'], 1]
    independent_cut = (matrix(originals[selected].findtext('pose')) @ local)[:3]
    assert np.linalg.norm(independent_cut - scene['cut_point_world']) < 1e-10
    result = {'baseline_id': scene['baseline_id'], 'source_world': str(source_path),
              'source_sha256': scene['source_world_sha256'], 'passed': True,
              'original_includes_preserved': len(originals), 'plant_instances_preserved': 60,
              'non_target_plant_instances_checked': len(other_models_checked),
              'selected_target_geometry': counts,
              'cut_transform_error_m': float(np.linalg.norm(independent_cut - scene['cut_point_world'])),
              'scope': 'serialized scene integrity only; physical harvest not tested'}
    (ROOT / 'validation/v4_harvest_world_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
