"""ROS-independent person detection and range estimation helpers (OpenCV DNN)."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

PERSON_CLASS_ID = 0  # COCO class index for 'person'


@dataclass
class Detection:
    """Axis-aligned bounding box in original image pixel coordinates."""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float

    @property
    def cx(self) -> float:
        """Horizontal box centre."""
        return (self.x1 + self.x2) / 2.0

    @property
    def height(self) -> float:
        """Box height in pixels."""
        return self.y2 - self.y1


@dataclass
class TargetEstimate:
    """Person position in the camera_link frame (x forward, y left)."""

    forward: float
    left: float
    method: str


def letterbox(image: np.ndarray, size: int) -> Tuple[np.ndarray, float]:
    """Resize keeping aspect ratio and pad to a size x size square (top-left aligned)."""
    h, w = image.shape[:2]
    scale = size / max(h, w)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[:new_h, :new_w] = resized
    return canvas, scale


def parse_yolo_output(
    output: np.ndarray,
    scale: float,
    img_w: int,
    img_h: int,
    conf_threshold: float,
    nms_threshold: float,
    class_id: int = PERSON_CLASS_ID,
) -> List[Detection]:
    """
    Decode a YOLOv8/YOLO11 ONNX output of shape (1, 4 + num_classes, N).

    Returns detections of `class_id` after NMS, sorted by descending score,
    with coordinates mapped back to the original (pre-letterbox) image.
    """
    preds = np.squeeze(output, axis=0).T  # (N, 4 + num_classes)
    scores = preds[:, 4 + class_id]
    keep = scores >= conf_threshold
    if not np.any(keep):
        return []
    preds = preds[keep]
    scores = scores[keep]

    cx, cy, bw, bh = (preds[:, i] / scale for i in range(4))
    x1 = np.clip(cx - bw / 2, 0, img_w - 1)
    y1 = np.clip(cy - bh / 2, 0, img_h - 1)
    x2 = np.clip(cx + bw / 2, 0, img_w - 1)
    y2 = np.clip(cy + bh / 2, 0, img_h - 1)

    boxes = [[float(a), float(b), float(c - a), float(d - b)]
             for a, b, c, d in zip(x1, y1, x2, y2)]
    indices = cv2.dnn.NMSBoxes(boxes, scores.tolist(), conf_threshold, nms_threshold)
    dets = [Detection(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
                      float(scores[i]))
            for i in np.array(indices).flatten()]
    dets.sort(key=lambda d: d.score, reverse=True)
    return dets


def estimate_target(
    det: Detection,
    img_w: int,
    img_h: int,
    hfov_rad: float,
    person_height: float,
    camera_height: float,
    min_depth: float,
    max_depth: float,
    edge_margin: float = 2.0,
) -> Optional[TargetEstimate]:
    """
    Estimate the person's position from a monocular bounding box.

    Assumes square pixels, a level camera and a person standing on the floor.
    - Whole body visible: range from apparent height (person_height).
    - Head cut off but feet visible: range from the feet's angle below the
      horizon (camera_height). This is the common case for a low hexapod
      camera at following distance, where the height method over-estimates.
    - Feet cut off: the person is very close, so report min_depth.
    """
    if det.height < 10.0:
        return None

    fx = img_w / (2.0 * math.tan(hfov_rad / 2.0))
    fy = fx
    cy0 = img_h / 2.0
    top_cut = det.y1 <= edge_margin
    bottom_cut = det.y2 >= img_h - 1 - edge_margin

    if not top_cut and not bottom_cut:
        depth, method = person_height * fy / det.height, 'height'
    elif not bottom_cut and det.y2 - cy0 > 1.0:
        depth, method = camera_height * fy / (det.y2 - cy0), 'ground'
    else:
        depth, method = min_depth, 'too_close'

    depth = max(min_depth, min(max_depth, depth))
    left = -((det.cx - img_w / 2.0) / fx) * depth
    return TargetEstimate(forward=depth, left=left, method=method)
