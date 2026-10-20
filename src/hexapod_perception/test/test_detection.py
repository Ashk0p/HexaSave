"""Unit tests for YOLO output decoding and range estimation."""

import math
import os

import cv2
from hexapod_perception.detection import (
    Detection, estimate_target, letterbox, parse_yolo_output)
import numpy as np
import pytest

HFOV = math.radians(62.2)
W, H = 640, 480
FX = W / (2 * math.tan(HFOV / 2))
MODEL = os.path.join(os.path.dirname(__file__), '..', 'models', 'yolo11n.onnx')


def make_output(rows, num_classes=80):
    """Build a (1, 4 + num_classes, N) array from (cx, cy, w, h, class_id, score) rows."""
    out = np.zeros((1, 4 + num_classes, len(rows)), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(rows):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + cls, i] = score
    return out


def test_letterbox_keeps_aspect_and_pads():
    img = np.full((H, W, 3), 255, dtype=np.uint8)
    padded, scale = letterbox(img, 640)
    assert padded.shape == (640, 640, 3)
    assert scale == pytest.approx(1.0)
    assert (padded[:480] == 255).all()
    assert (padded[480:] == 114).all()


def test_parse_maps_back_to_image_coordinates():
    scale = 320 / 640  # letterboxed to 320 input
    out = make_output([(160, 100, 40, 120, 0, 0.9)])
    dets = parse_yolo_output(out, scale, W, H, 0.4, 0.45)
    assert len(dets) == 1
    d = dets[0]
    assert (d.x1, d.y1, d.x2, d.y2) == pytest.approx((280, 80, 360, 320))
    assert d.score == pytest.approx(0.9)


def test_parse_ignores_low_confidence_and_other_classes():
    out = make_output([(100, 100, 50, 50, 0, 0.2),    # person, low score
                       (300, 300, 50, 50, 2, 0.95)])  # car
    assert parse_yolo_output(out, 1.0, W, H, 0.4, 0.45) == []


def test_parse_nms_and_sorting():
    out = make_output([(100, 200, 60, 200, 0, 0.6),
                       (102, 202, 60, 200, 0, 0.8),   # overlaps the first
                       (500, 200, 60, 200, 0, 0.7)])
    dets = parse_yolo_output(out, 1.0, W, H, 0.4, 0.45)
    assert [round(d.score, 2) for d in dets] == [0.8, 0.7]


def test_parse_clips_to_image():
    out = make_output([(10, 10, 100, 100, 0, 0.9)])
    d = parse_yolo_output(out, 1.0, W, H, 0.4, 0.45)[0]
    assert d.x1 == 0 and d.y1 == 0


def estimate(det, **kw):
    args = {'img_w': W, 'img_h': H, 'hfov_rad': HFOV, 'person_height': 1.7,
            'camera_height': 0.15, 'min_depth': 0.5, 'max_depth': 6.0}
    args.update(kw)
    return estimate_target(det, **args)


def test_full_body_uses_height():
    pix_h = 1.7 * FX / 4.0  # person 4 m away
    det = Detection(300, 240 - pix_h / 2, 340, 240 + pix_h / 2, 0.9)
    t = estimate(det)
    assert t.method == 'height'
    assert t.forward == pytest.approx(4.0)
    assert t.left == pytest.approx(0.0)


def test_head_cut_uses_ground_plane():
    # Feet at 1.2 m with the camera 0.15 m above the floor.
    feet_y = H / 2 + 0.15 * FX / 1.2
    det = Detection(300, 0, 340, feet_y, 0.9)
    t = estimate(det)
    assert t.method == 'ground'
    assert t.forward == pytest.approx(1.2)


def test_feet_cut_is_too_close():
    det = Detection(200, 50, 400, H - 1, 0.9)
    t = estimate(det)
    assert t.method == 'too_close'
    assert t.forward == 0.5


def test_lateral_sign_person_on_left_is_positive_y():
    pix_h = 1.7 * FX / 3.0
    det = Detection(50, 240 - pix_h / 2, 110, 240 + pix_h / 2, 0.9)
    t = estimate(det)
    assert t.left > 0
    assert t.left == pytest.approx((W / 2 - 80) / FX * 3.0)


def test_tiny_box_rejected():
    assert estimate(Detection(10, 100, 15, 105, 0.9)) is None


def test_depth_clamped():
    det = Detection(318, 239, 322, 255, 0.9)  # 16 px tall -> ~56 m
    assert estimate(det).forward == 6.0


@pytest.mark.skipif(not os.path.exists(MODEL), reason='ONNX model not present')
def test_model_runs_with_opencv_dnn():
    net = cv2.dnn.readNetFromONNX(MODEL)
    img = np.zeros((H, W, 3), dtype=np.uint8)
    padded, scale = letterbox(img, 640)
    net.setInput(cv2.dnn.blobFromImage(padded, 1 / 255.0, swapRB=True))
    out = net.forward()
    assert out.shape == (1, 84, 8400)
    assert parse_yolo_output(out, scale, W, H, 0.4, 0.45) == []
