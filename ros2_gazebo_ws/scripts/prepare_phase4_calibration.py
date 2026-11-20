#!/usr/bin/env python3
"""Write eye-in-hand transforms from the assembled Gazebo TF tree."""
import argparse
from pathlib import Path

from simulation_config import workspace
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.utilities import remove_ros_args
from tomato_sim_adapters.arm_kinematics import SDK_BASE_Z_OFFSET


def calibration_matrix(transform, pose_reference):
    q = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [transform.translation.x, transform.translation.y,
                      transform.translation.z]
    if pose_reference == 'sdk_flange':
        matrix[2, 3] -= SDK_BASE_Z_OFFSET
    return matrix


class CalibrationWriter(Node):
    def __init__(self, output, pose_reference='tcp'):
        super().__init__('prepare_phase4_calibration')
        self.output = Path(output)
        self.pose_reference = pose_reference
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def run(self):
        for side in ('left', 'right'):
            source = f'cam_{side}_color_optical_frame'
            deadline = time.monotonic() + 30.0
            transform = None
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    transform = self.buffer.lookup_transform(
                        'arm_base_link', source, rclpy.time.Time()).transform
                    break
                except TransformException:
                    continue
            if transform is None:
                raise RuntimeError(f'Missing TF arm_base_link <- {source}')
            matrix = calibration_matrix(transform, self.pose_reference)
            path = self.output / side / 'calibration_results.txt'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('\n'.join(','.join(f'{value:.12g}' for value in row)
                                      for row in matrix) + '\n')
            print(f'{side} optical transform for {self.pose_reference}\n{matrix}')


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', nargs='?', type=Path,
                        default=workspace() / 'validation/phase4/calibration')
    parser.add_argument('--pose-reference', choices=('tcp', 'sdk_flange'), default='tcp')
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main():
    args = parse_arguments()
    rclpy.init()
    node = CalibrationWriter(args.output, args.pose_reference)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
