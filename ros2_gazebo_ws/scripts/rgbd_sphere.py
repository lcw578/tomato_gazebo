"""Measure the magenta calibration sphere using registered RGB and depth."""
import cv2
import numpy as np
from scipy.optimize import least_squares


def measure_sphere(rgb, depth, intrinsics, radius, units_per_metre):
    color = rgb.astype(float)
    mask = ((color[:, :, 0] > 60) & (color[:, :, 2] > 60)
            & (color[:, :, 0] > 1.8 * color[:, :, 1] + 20)
            & (color[:, :, 2] > 1.8 * color[:, :, 1] + 20))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if count < 2:
        raise ValueError('Calibration sphere is absent from the RGB image')
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = cv2.erode((labels == component).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    z = depth.astype(float) / units_per_metre
    v, u = np.nonzero(mask & np.isfinite(z) & (z > 0))
    if len(u) < 50:
        raise ValueError('Insufficient valid sphere depth samples')
    fx, fy, cx, cy = intrinsics[0, 0], intrinsics[1, 1], intrinsics[0, 2], intrinsics[1, 2]
    points = np.column_stack(((u - cx) * z[v, u] / fx, (v - cy) * z[v, u] / fy, z[v, u]))
    seed = np.median(points, axis=0)
    seed += radius * seed / np.linalg.norm(seed)
    fit = least_squares(lambda center: np.linalg.norm(points - center, axis=1) - radius,
                        seed, loss='soft_l1', f_scale=0.001)
    residuals = np.linalg.norm(points - fit.x, axis=1) - radius
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    if not fit.success or rms > 0.003:
        raise ValueError(f'Invalid sphere fit, surface RMS {rms} m')
    return {'center_camera_m': fit.x, 'rms_surface_error_m': rms,
            'pixel_center': [float(np.mean(u)), float(np.mean(v))], 'depth_samples': len(u)}
