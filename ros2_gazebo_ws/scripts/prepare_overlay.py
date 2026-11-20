#!/usr/bin/env python3
"""Validate external inputs and generate a colcon overlay of the real robot packages."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from simulation_config import load_configuration, read_world, sha256_file, workspace


SHARED_PACKAGES = {
    'xc_interfaces': 'xc_interfaces',
    'vision_tomato_interfaces': 'vision_packages/vision_tomato_interfaces',
    'vision_pose_generate': 'vision_packages/vision_pose_generate',
}
PYTHON_PACKAGES = ('xc_robot_main_control', 'closed_loop_feedback_detection')
AUBO_PACKAGE = 'xc_aubo_arm_control/aubo_description'


def required_file(path):
    if not path.is_file() or not path.stat().st_size:
        raise ValueError(f'Required file is missing or empty: {path}')
    return path


def packaging(source):
    manifest = required_file(source / 'package.xml').read_text().replace(
        '  <depend>realsense2_camera</depend>\n', '')
    setup = required_file(source / 'setup.py').read_text()
    if source.name == 'xc_robot_main_control':
        manifest = manifest.replace('  <depend>xc_interfaces</depend>',
            '  <depend>xc_interfaces</depend>\n  <depend>vision_tomato_interfaces</depend>')
        setup = setup.replace("['setuptools','xc_interfaces']", "['setuptools']")
    setup, count = re.subn(r"        \('share/' \+ package_name \+ '/weights', \[.*?\]\),\n",
                           "        ('share/' + package_name + '/weights', sorted(glob('weights/*.pt'))),\n",
                           setup, flags=re.S)
    if count != 1:
        raise ValueError(f'Unexpected weights setup in {source / "setup.py"}')
    return {'package.xml': manifest, 'setup.py': 'from glob import glob\n' + setup}


def plan_overlay(config):
    root = workspace()
    robot = Path(config['robot_root']).resolve()
    source = robot / 'src'
    assets = Path(config['asset_root']).resolve()
    world = required_file(Path(config['world']).resolve())
    _, sdf_world = read_world(world)
    if sdf_world.get('name') != config['baseline_id']:
        raise ValueError(f'Expected the configured baseline {config["baseline_id"]} in {world}')
    weight = required_file(Path(config['yolo_weights']).resolve())
    for name in ('ros_python', 'yolo_python'):
        python = required_file(Path(config[name]))
        if not os.access(python, os.X_OK):
            raise ValueError(f'{name} is not executable: {python}')

    model_root = assets / 'assets/models'
    for uri in sorted({node.text for node in sdf_world.findall('include/uri') if node.text}):
        if uri.startswith('model://'):
            model = uri.removeprefix('model://')
            if not (model_root / model / 'model.config').is_file():
                required_file(Path('/usr/share/gazebo-11/models') / model / 'model.config')

    links = {}
    metadata = {}
    for name, relative in SHARED_PACKAGES.items():
        package = source / relative
        required_file(package / 'package.xml')
        links[Path(name)] = package

    for name in PYTHON_PACKAGES:
        package = source / name
        for filename, contents in packaging(package).items():
            metadata[Path(name, filename)] = contents
        for item in package.iterdir():
            if item.name not in ('package.xml', 'setup.py', 'weights', '__pycache__'):
                links[Path(name, item.name)] = item
        for item in (package / 'weights').glob('*.pt'):
            links[Path(name, 'weights', item.name)] = item
    links[Path('xc_robot_main_control/weights/best.pt')] = weight

    cad_meshes = root / 'src/tomato_sim_description/meshes'
    for name in ('chassis_0', 'lift_0', 'basket_0', 'cutter_0', 'cutter_1', 'cutter_2',
                 'gear_0', 'left_finger_0', 'right_finger_0'):
        required_file(cad_meshes / f'{name}.stl')

    aubo = source / AUBO_PACKAGE
    for name in ('mesh', 'urdf'):
        if not (aubo / name).is_dir():
            raise ValueError(f'Required AUBO directory is missing: {aubo / name}')
        links[Path('aubo_description', name)] = aubo / name
    for name in ('CMakeLists.txt', 'package.xml', 'xacro'):
        links[Path('aubo_description', name)] = root / 'src/aubo_description' / name

    source_files = {}
    reference = json.loads((root / 'src/tomato_sim_gazebo/config/robot_source_reference.json').read_text())
    for relative, hashes in reference['files'].items():
        path = required_file(robot / relative)
        digest = sha256_file(path)
        source_files[relative] = {
            'sha256': digest,
            'matches_delivery': digest == hashes['delivery_sha256'],
            'matches_audited_workspace': digest == hashes['workspace_sha256'],
        }
    revision = None
    if (robot / '.git').exists():
        revision = subprocess.check_output(['git', '-C', str(robot), 'rev-parse', 'HEAD'],
                                           text=True).strip()
    report = {
        'recorded_at': datetime.now(timezone.utc).isoformat(),
        'robot_source': {
            'root': str(robot), 'git_head': revision, 'delivery_reference': reference['delivery'],
            'files': source_files,
        },
        'map': {'asset_root': str(assets), 'baseline_id': config['baseline_id'],
                'world': str(world), 'world_name': sdf_world.get('name'), 'sha256': sha256_file(world)},
        'yolo_weights': {'path': str(weight), 'sha256': sha256_file(weight)},
        'python': {name: str(Path(config[name]).absolute()) for name in ('ros_python', 'yolo_python')},
    }
    return links, metadata, report


def prepare(links, metadata, report, overlay):
    for relative, target in links.items():
        link = overlay / 'src' / relative
        link.parent.mkdir(parents=True, exist_ok=True)
        if not link.is_symlink() and link.exists():
            raise ValueError(f'Expected a generated symlink at {link}')
        if link.is_symlink() and link.readlink() == target:
            continue
        with tempfile.TemporaryDirectory(dir=link.parent) as directory:
            replacement = Path(directory) / 'link'
            replacement.symlink_to(target, target_is_directory=target.is_dir())
            replacement.replace(link)

    for relative, contents in metadata.items():
        path = overlay / 'src' / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    # Keep the generated wrappers in sync when a different checkout lacks optional files.
    for name in PYTHON_PACKAGES:
        package = overlay / 'src' / name
        for old_link in [*package.iterdir(), *(package / 'weights').glob('*.pt')]:
            if old_link.is_symlink() and old_link.relative_to(overlay / 'src') not in links:
                old_link.unlink()
    with tempfile.NamedTemporaryFile(mode='w', dir=overlay, delete=False) as record:
        json.dump(report, record, indent=2)
        record.write('\n')
    Path(record.name).replace(overlay / 'dependencies.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Validate and print provenance only.')
    args = parser.parse_args()
    try:
        links, metadata, report = plan_overlay(load_configuration())
        if args.check:
            print(json.dumps(report, indent=2))
        else:
            overlay = workspace() / '.overlay'
            prepare(links, metadata, report, overlay)
            print(f'Dependency record: {overlay / "dependencies.json"}')
    except (OSError, ValueError, ET.ParseError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
