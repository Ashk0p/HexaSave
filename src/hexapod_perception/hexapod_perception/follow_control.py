"""ROS-independent proportional controller for following a target."""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class FollowGains:
    """Controller tuning."""

    target_dist: float = 1.2
    kp_linear: float = 0.4
    kp_angular: float = 0.8
    dist_deadband: float = 0.08
    angle_deadband: float = 0.05
    max_linear: float = 0.3
    max_angular: float = 0.6


def _deadband(error: float, band: float) -> float:
    if abs(error) <= band:
        return 0.0
    return error - math.copysign(band, error)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def compute_cmd(forward: float, left: float, gains: FollowGains) -> Tuple[float, float]:
    """Return (linear.x, angular.z) to approach a target at (forward, left)."""
    distance = math.hypot(forward, left)
    angle_error = math.atan2(left, forward)
    angular = _clamp(gains.kp_angular * _deadband(angle_error, gains.angle_deadband),
                     gains.max_angular)
    linear = _clamp(gains.kp_linear * _deadband(distance - gains.target_dist,
                                                gains.dist_deadband),
                    gains.max_linear)
    return linear, angular
