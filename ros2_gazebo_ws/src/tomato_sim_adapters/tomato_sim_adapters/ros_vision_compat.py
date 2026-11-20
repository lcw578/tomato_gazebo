"""Pure-Python ROS image and pinhole-camera adapters for the Conda vision runtime."""
from array import array
import sys
import types

import numpy as np
from sensor_msgs.msg import Image


def _image_dtype(encoding):
    return {
        'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3),
        'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1),
        '16UC1': ('<u2', 1), '32FC1': ('<f4', 1),
    }[encoding]


class TomatoSimCvBridge:
    def imgmsg_to_cv2(self, message, desired_encoding='passthrough'):
        encoding = message.encoding if desired_encoding == 'passthrough' else desired_encoding
        dtype, channels = _image_dtype(message.encoding)
        shape = ((message.height, message.width, channels) if channels == 3
                 else (message.height, message.width))
        itemsize = np.dtype(dtype).itemsize
        row_stride = message.step or message.width * channels * itemsize
        strides = ((row_stride, channels * itemsize, itemsize) if channels == 3
                   else (row_stride, itemsize))
        image = np.ndarray(shape, dtype=dtype, buffer=message.data, strides=strides).copy()
        if message.encoding == 'rgb8' and encoding == 'bgr8':
            return image[..., ::-1].copy()
        if message.encoding == 'bgr8' and encoding == 'rgb8':
            return image[..., ::-1].copy()
        if encoding not in ('passthrough', message.encoding, 'bgr8', 'rgb8'):
            raise ValueError(f'Unsupported conversion {message.encoding} -> {encoding}')
        return image

    def cv2_to_imgmsg(self, image, encoding='passthrough'):
        value = np.ascontiguousarray(image)
        if encoding == 'passthrough':
            encoding = {np.dtype(np.uint8): 'mono8', np.dtype('<u2'): '16UC1',
                        np.dtype('<f4'): '32FC1'}.get(value.dtype)
            if value.ndim == 3 and value.shape[2] == 3:
                encoding = 'bgr8'
        if encoding is None:
            raise ValueError(f'Cannot infer image encoding for dtype {value.dtype}')
        channels = value.shape[2] if value.ndim == 3 else 1
        message = Image()
        message.height, message.width = value.shape[:2]
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = value.strides[0]
        message.data = array('B', value.tobytes())
        return message


class _Intrinsics:
    width = 0
    height = 0
    fx = 0.0
    fy = 0.0
    ppx = 0.0
    ppy = 0.0
    model = 0
    coeffs = [0.0] * 5


def _deproject(intrinsics, pixel, depth):
    u, v = pixel
    return [(u - intrinsics.ppx) * depth / intrinsics.fx,
            (v - intrinsics.ppy) * depth / intrinsics.fy,
            depth]


def install_cv_bridge():
    bridge_module = types.ModuleType('cv_bridge')
    bridge_module.CvBridge = TomatoSimCvBridge
    sys.modules['cv_bridge'] = bridge_module


def install_realsense_pinhole():
    rs_module = types.ModuleType('pyrealsense2')
    rs_module.intrinsics = _Intrinsics
    rs_module.distortion = types.SimpleNamespace(brown_conrady=0, kannala_brandt4=1)
    rs_module.rs2_deproject_pixel_to_point = _deproject
    sys.modules['pyrealsense2'] = rs_module
