"""Check RGB-D sphere fitting against independent ray/sphere intersections."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rgbd_sphere import measure_sphere


class SphereMeasurementTest(unittest.TestCase):
    def test_reconstructs_sphere_from_quantized_depth(self):
        camera = np.array([[400., 0, 160], [0, 400., 120], [0, 0, 1]])
        center = np.array([0.02, -0.01, 0.65])
        radius = 0.035
        v, u = np.indices((240, 320))
        rays = np.stack(((u - 160) / 400, (v - 120) / 400, np.ones_like(u)), axis=-1)
        a = np.sum(rays ** 2, axis=-1)
        b = rays @ center
        discriminant = b ** 2 - a * (center @ center - radius ** 2)
        mask = discriminant >= 0
        depth = np.zeros((240, 320), dtype=np.uint16)
        depth[mask] = np.rint((b[mask] - np.sqrt(discriminant[mask])) / a[mask] * 1000)
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        rgb[mask] = [240, 0, 240]
        result = measure_sphere(rgb, depth, camera, radius, 1000)
        self.assertLess(np.linalg.norm(result['center_camera_m'] - center), 0.001)
        self.assertLess(result['rms_surface_error_m'], 0.001)

    def test_rejects_missing_sphere(self):
        with self.assertRaises(ValueError):
            measure_sphere(np.zeros((40, 40, 3), dtype=np.uint8),
                           np.zeros((40, 40)), np.eye(3), 0.035, 1000)


if __name__ == '__main__':
    unittest.main()
