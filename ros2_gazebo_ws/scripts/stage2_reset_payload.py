#!/usr/bin/env python3
"""Restore the target fruit locally between harvest acceptance runs."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from scipy.spatial.transform import Rotation
from xc_interfaces.srv import MoveJoints
import yaml

from phase3_checks import Phase3Checks, WORKSPACE
from tomato_sim_adapters.arm_kinematics import JOINTS, OEM_HOME
from tomato_sim_adapters.harvest_physics_arbiter import pose_msg


def reset_payload(node, exit_basket, scene_file=None, motion_config=None):
    """Run with the harvest arbiter stopped, then start a fresh arbiter."""
    scene_file = Path(scene_file) if scene_file else WORKSPACE / 'validation/phase3/phase3_scene.json'
    scene = json.loads(scene_file.read_text())
    if exit_basket:
        config = Path(motion_config) if motion_config else (
            WORKSPACE / 'src/tomato_sim_gazebo/config/stage2_truth_motion.yaml')
        parameters = yaml.safe_load(config.read_text())['/**']['ros__parameters']
        points = np.asarray(parameters['box_waypoints_rad']).reshape((-1, 6))
        node.wait_for(lambda: all(name in node.joints for name in JOINTS), 'arm joint states')
        current = np.array([node.joints[name] for name in JOINTS])
        if np.max(np.abs(current - points[-1])) > 0.08:
            raise RuntimeError('Arm is outside the verified basket-exit starting pose')
        node.command_gripper('open')
        for index, point in enumerate((points[-2], OEM_HOME)):
            node.call(node.move_joints, MoveJoints.Request(joints=point.tolist()),
                      f'empty basket exit waypoint {index}', timeout=240)

    models = node.create_client(GetModelList, '/get_model_list')
    delete = node.create_client(DeleteEntity, '/delete_entity')
    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    existing = node.call(models, GetModelList.Request(), 'list local models').model_names
    for name in (scene['target_model'], scene['target_model'] + '_released'):
        if name in existing:
            node.call(delete, DeleteEntity.Request(name=name), f'delete {name}', timeout=30)
            node.wait_for(lambda: name not in node.call(
                models, GetModelList.Request(), 'wait for local model deletion').model_names,
                f'Gazebo removal of {name}', timeout=30)

    plant = np.array([float(value) for value in scene['plant_include_pose'].split()])
    fruit = np.array([float(value) for value in scene['fruit_local_pose'].split()])
    plant_rotation = Rotation.from_euler('xyz', plant[3:])
    fruit_rotation = Rotation.from_euler('xyz', fruit[3:])
    initial_pose = (plant[:3] + plant_rotation.apply(fruit[:3]),
                    (plant_rotation * fruit_rotation).as_quat())
    root = ET.parse(scene_file.parent / scene['release_sdf']).getroot()
    root.find('model').set('name', scene['target_model'])
    node.call(spawn, SpawnEntity.Request(
        name=scene['target_model'], xml=ET.tostring(root, encoding='unicode'),
        initial_pose=pose_msg(initial_pose), reference_frame='world'),
        'restore target at its original plant pose', timeout=30)
    print('Local target restored. Start a fresh harvest arbiter before the next run.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exit-basket', action='store_true',
                        help='Move the empty arm from the verified low release pose to OEM home')
    parser.add_argument('--scene-file', type=Path)
    parser.add_argument('--motion-config', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Phase3Checks(Path('validation/stage2'), args.scene_file)
    try:
        reset_payload(node, args.exit_basket, args.scene_file, args.motion_config)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
