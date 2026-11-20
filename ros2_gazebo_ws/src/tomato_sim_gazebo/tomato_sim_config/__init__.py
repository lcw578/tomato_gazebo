"""Resolve the shared Gazebo configuration for local scripts and ROS launch."""
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET


def workspace():
    root = os.environ.get('TOMATO_SIM_WORKSPACE')
    return Path(root).expanduser().resolve() if root else Path(__file__).resolve().parents[3]


def load_configuration():
    path = workspace() / 'src/tomato_sim_gazebo/config/simulation_baseline.json'
    config = json.loads(path.read_text())
    for name, spec in config.pop('dependencies').items():
        value = os.environ.get(spec['env'])
        if name == 'yolo_python' and not value:
            value = os.environ.get('TOMATO_SIM_PYTHON')
        # Preserve the virtualenv entry point, including its bin/python symlink.
        path = workspace() / Path(value or spec['default']).expanduser()
        config[name] = str(path.absolute() if name.endswith('_python') else path.resolve())
    config['world'] = str(Path(config['asset_root']) / config['world'])
    return config


def read_world(path):
    tree = ET.parse(path)
    worlds = tree.getroot().findall('world')
    if len(worlds) != 1 or not worlds[0].get('name'):
        raise ValueError(f'Expected one named SDF world in {path}')
    return tree, worlds[0]


def sha256_file(path):
    with Path(path).open('rb') as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def assembly_launch_defaults(config):
    return {
        'world': config['world'],
        'arm_base_yaw_deg': str(config['arm_base_yaw_deg']),
        'lift_height': str(config['lift_height_m']),
        'camera_angle_deg': str(config['camera_angle_deg']),
        'camera_tilt_deg': str(config['camera_tilt_deg']),
        'camera_hfov_deg': str(config['camera_hfov_deg']),
    }


def harvest_launch_defaults(config):
    station = config['candidate_harvest_station']
    return {
        **assembly_launch_defaults(config),
        'world': str(harvest_world_path(config)),
        'slide_initial': os.environ.get('SIM_RAIL_STATION_M') or str(station['rail_m']),
        'lift_height': os.environ.get('SIM_LIFT_HEIGHT_M') or str(station['lift_m']),
        'camera_angle_deg': os.environ.get('SIM_CAMERA_ANGLE_DEG')
                            or str(station['right_camera_angle_deg']),
        'camera_tilt_deg': os.environ.get('SIM_CAMERA_TILT_DEG')
                           or str(station['right_camera_tilt_deg']),
        'camera_hfov_deg': os.environ.get('SIM_CAMERA_HFOV_DEG') or str(config['camera_hfov_deg']),
    }


def harvest_world_path(config, output=None):
    folder = Path(output) if output is not None else workspace() / 'validation/phase3'
    return folder / f"{config['baseline_id']}_phase3.world"


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('key', choices=['asset_root', 'baseline_id', 'world',
                                       'harvest_world', 'harvest_rail', 'ros_python',
                                       'yolo_python', 'yolo_weights'])
    args = parser.parse_args()
    config = load_configuration()
    if args.key == 'harvest_world':
        value = harvest_world_path(config)
    elif args.key == 'harvest_rail':
        value = harvest_launch_defaults(config)['slide_initial']
    else:
        value = config[args.key]
    print(value)
