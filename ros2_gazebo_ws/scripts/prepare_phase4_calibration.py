#!/usr/bin/env python3
"""Write eye-in-hand transforms from the assembled Gazebo TF tree."""
from pathlib import Path

from simulation_config import workspace
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener


class CalibrationWriter(Node):
    def __init__(self, output):
        super().__init__('prepare_phase4_calibration')
        self.output = Path(output)
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
            q = transform.rotation
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            matrix[:3, 3] = [transform.translation.x, transform.translation.y,
                             transform.translation.z]
            path = self.output / side / 'calibration_results.txt'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('\n'.join(','.join(f'{value:.12g}' for value in row)
                                      for row in matrix) + '\n')
            print(f'{side} T_arm_base_from_optical:\n{matrix}')


def main():
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else workspace() / 'validation/phase4/calibration'
    rclpy.init()
    node = CalibrationWriter(output)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
