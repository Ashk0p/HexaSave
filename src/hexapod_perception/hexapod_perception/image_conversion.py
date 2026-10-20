"""
sensor_msgs/Image <-> numpy conversion without cv_bridge.

The ROS cv_bridge binary is built against the system OpenCV 4.x, whose type
codes differ from OpenCV 5.x (cv2_to_imgmsg raises KeyError), so the
perception nodes convert images with numpy directly.
"""

import cv2
import numpy as np
from sensor_msgs.msg import Image

_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}
_TO_BGR = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
           'rgba8': cv2.COLOR_RGBA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Convert an 8-bit Image message to a BGR uint8 array (copy)."""
    channels = _CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    row_bytes = msg.width * channels
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :row_bytes].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'bgr8':
        return img.copy()
    return cv2.cvtColor(img if channels > 1 else img[:, :, 0], _TO_BGR[msg.encoding])


def bgr_to_imgmsg(image: np.ndarray) -> Image:
    """Convert a BGR uint8 array to a bgr8 Image message (header left empty)."""
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected an HxWx3 uint8 BGR image')
    msg = Image()
    msg.height, msg.width = image.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg
