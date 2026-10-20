"""Unit tests for Image message <-> numpy conversion."""

from hexapod_perception.image_conversion import bgr_to_imgmsg, imgmsg_to_bgr
import numpy as np
import pytest
from sensor_msgs.msg import Image


def make_msg(arr, encoding, pad=0):
    h, w = arr.shape[:2]
    ch = 1 if arr.ndim == 2 else arr.shape[2]
    rows = arr.reshape(h, w * ch)
    if pad:
        rows = np.hstack([rows, np.zeros((h, pad), np.uint8)])
    msg = Image()
    msg.height, msg.width, msg.encoding = h, w, encoding
    msg.step = w * ch + pad
    msg.data = rows.tobytes()
    return msg


@pytest.fixture
def bgr():
    rng = np.random.default_rng(0)
    return rng.integers(0, 255, (4, 5, 3), dtype=np.uint8)


def test_roundtrip_bgr8(bgr):
    msg = bgr_to_imgmsg(bgr)
    assert msg.encoding == 'bgr8' and msg.step == 15
    np.testing.assert_array_equal(imgmsg_to_bgr(msg), bgr)


def test_rgb8_is_channel_swapped(bgr):
    out = imgmsg_to_bgr(make_msg(bgr[:, :, ::-1].copy(), 'rgb8'))
    np.testing.assert_array_equal(out, bgr)


def test_row_padding_is_ignored(bgr):
    out = imgmsg_to_bgr(make_msg(bgr, 'bgr8', pad=7))
    np.testing.assert_array_equal(out, bgr)


def test_rgba8_and_mono8(bgr):
    rgba = np.dstack([bgr[:, :, ::-1], np.full(bgr.shape[:2], 255, np.uint8)])
    np.testing.assert_array_equal(imgmsg_to_bgr(make_msg(rgba, 'rgba8')), bgr)
    mono = bgr[:, :, 0].copy()
    out = imgmsg_to_bgr(make_msg(mono, 'mono8'))
    assert out.shape == (4, 5, 3)
    np.testing.assert_array_equal(out[:, :, 2], mono)


def test_unsupported_encoding_raises(bgr):
    with pytest.raises(ValueError):
        imgmsg_to_bgr(make_msg(bgr, 'yuv422'))
