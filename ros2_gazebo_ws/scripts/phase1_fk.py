#!/usr/bin/env python3
"""Compute independent TCP references in MuJoCo and from the vendor URDF."""
import json
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

WORKSPACE = Path(__file__).resolve().parents[1]
MJCF = WORKSPACE.parent / 'assets/fk_reference/tomato_picker_standalone/tomato_picker_standalone.xml'
AUBO = Path(get_package_share_directory('aubo_description')) / 'urdf/aubo_i5.urdf'
JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')
POSES = {
    'zero': [0.0] * 6,
    'pose_a': [0.20, -0.22, 0.30, 0.10, -0.16, 0.18],
    'pose_b': [-0.22, 0.24, -0.30, -0.12, 0.16, -0.16],
}


def vector(value):
    return np.fromstring(value, sep=' ')


def pose(xyz, rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def encode(matrix):
    return {'xyz': matrix[:3, 3].tolist(),
            'xyzw': Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()}


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    aubo_joints = {joint.get('name'): joint for joint in ET.parse(AUBO).getroot().findall('joint')}
    tool = ET.parse(MJCF).find(".//body[@name='wrist3_Link']/body[@name='base_link']")
    # Original contact_tool_frame locates the blade tip in cutter CAD coordinates.
    # Lateral calibration follows TCP_BASE in the old tools/scene_assets.py.
    depth = vector(tool.find("body[@name='gear_link']").get('pos'))[2] - 0.08025
    contact_point = np.array([-0.0049, -0.0044, depth])
    w, x, y, z = vector(tool.get('quat'))
    tool_mount = pose(vector(tool.get('pos')), Rotation.from_quat([x, y, z, w]))
    cutter_tcp = pose(contact_point, Rotation.identity())
    results = {}
    for label, positions in POSES.items():
        for name, angle in zip(JOINTS, positions):
            data.qpos[model.joint(name).qposadr[0]] = angle
        mujoco.mj_kinematics(model, data)
        base = data.body('arm_base_mount')
        cutter = data.body('base_link')
        world_base = pose(base.xpos, Rotation.from_matrix(base.xmat.reshape(3, 3)))
        world_cutter = pose(cutter.xpos, Rotation.from_matrix(cutter.xmat.reshape(3, 3)))
        aubo = np.eye(4)
        for name, angle in zip(JOINTS, positions):
            joint = aubo_joints[name]
            origin = joint.find('origin')
            axis = vector(joint.find('axis').get('xyz'))
            aubo = (aubo @ pose(vector(origin.get('xyz')),
                                Rotation.from_euler('xyz', vector(origin.get('rpy'))))
                    @ pose(np.zeros(3), Rotation.from_rotvec(axis * angle)))
        results[label] = {
            'q_rad': positions,
            'mujoco': encode(np.linalg.solve(world_base, world_cutter) @ cutter_tcp),
            'aubo_i5': encode(aubo @ tool_mount @ cutter_tcp),
        }
    Path(sys.argv[1]).write_text(json.dumps({'joints': JOINTS, 'poses': results}, indent=2) + '\n')


if __name__ == '__main__':
    main()
