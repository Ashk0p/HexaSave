"""Draw velocity command and target info onto a camera frame for human review."""

import math
from typing import Optional, Tuple

import cv2
import numpy as np

GREEN = (80, 200, 80)
RED = (60, 60, 230)
GREY = (170, 170, 170)
WHITE = (255, 255, 255)
AMBER = (0, 190, 255)
PANEL_H = 64


def _text(img: np.ndarray, text: str, org: Tuple[int, int], color=WHITE) -> None:
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def draw_velocity_gauge(img: np.ndarray, center: Tuple[int, int], radius: int,
                        lin: float, ang: float, max_lin: float, max_ang: float) -> None:
    """
    Top-down gauge: up = forward. Arrow length ~ |linear.x|, tilt ~ angular.z.

    A turn-in-place command (no linear part) is drawn as an arc, counter-clockwise
    for a left turn (positive angular.z).
    """
    cx, cy = center
    cv2.circle(img, center, radius + 6, (32, 32, 32), -1, cv2.LINE_AA)  # readable on any bg
    cv2.circle(img, center, radius, GREY, 1, cv2.LINE_AA)
    cv2.circle(img, center, 2, GREY, -1)
    lin_frac = max(-1.0, min(1.0, lin / max_lin)) if max_lin > 0 else 0.0
    ang_frac = max(-1.0, min(1.0, ang / max_ang)) if max_ang > 0 else 0.0

    if abs(lin_frac) > 1e-3:
        tilt = ang_frac * math.radians(60)  # positive (left) tilts the arrow left
        length = radius * abs(lin_frac)
        direction = 1.0 if lin_frac > 0 else -1.0
        tip = (int(cx - math.sin(tilt) * length * direction),
               int(cy - math.cos(tilt) * length * direction))
        cv2.arrowedLine(img, center, tip, GREEN if lin_frac > 0 else RED, 3,
                        cv2.LINE_AA, tipLength=0.3)
    elif abs(ang_frac) > 1e-3:
        span = int(abs(ang_frac) * 270)
        r = int(radius * 0.7)
        # OpenCV ellipse angles run clockwise from +x; -90 is straight up.
        if ang_frac > 0:   # counter-clockwise from the top
            start, end = -90 - span, -90
            tip_deg = -90 - span
        else:
            start, end = -90, -90 + span
            tip_deg = -90 + span
        cv2.ellipse(img, center, (r, r), 0, start, end, AMBER, 3, cv2.LINE_AA)
        t = math.radians(tip_deg)
        cv2.circle(img, (int(cx + r * math.cos(t)), int(cy + r * math.sin(t))), 5, AMBER, -1)


def draw_overlay(
    image: np.ndarray,
    lin: Optional[float],
    ang: Optional[float],
    cmd_age: Optional[float],
    target: Optional[Tuple[float, float]],
    stamp_s: float,
    max_lin: float = 0.3,
    max_ang: float = 0.6,
    stale_after: float = 1.0,
) -> np.ndarray:
    """
    Return a copy of `image` with a review panel appended below it.

    lin/ang/cmd_age are None when no /cmd_vel has been received yet;
    target is (forward, left) in metres or None.
    """
    h, w = image.shape[:2]
    out = np.zeros((h + PANEL_H, w, 3), dtype=np.uint8)
    out[:h] = image
    out[h:] = (32, 32, 32)

    if lin is None or ang is None:
        cmd_line, color = 'cmd_vel: none received', GREY
        lin_v, ang_v = 0.0, 0.0
    else:
        lin_v, ang_v = lin, ang
        moving = abs(lin) > 1e-3 or abs(ang) > 1e-3
        cmd_line = f'cmd_vel  lin.x {lin:+.2f} m/s   ang.z {ang:+.2f} rad/s'
        if cmd_age is not None and cmd_age > stale_after:
            cmd_line += f'   STALE {cmd_age:.1f}s'
            color = AMBER
        else:
            color = GREEN if moving else WHITE

    if target is None:
        target_line = 'target: none'
    else:
        fwd, left = target
        side = 'left' if left >= 0 else 'right'
        target_line = f'target: {fwd:.2f} m ahead, {abs(left):.2f} m {side}'

    _text(out, cmd_line, (8, h + 22), color)
    _text(out, target_line, (8, h + 44), WHITE if target else GREY)
    _text(out, f't={stamp_s:.2f}', (8, h + PANEL_H - 4), GREY)

    radius = 26
    draw_velocity_gauge(out, (w - radius - 10, h - radius - 10), radius,
                        lin_v, ang_v, max_lin, max_ang)
    return out
