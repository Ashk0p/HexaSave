"""Unit tests for the person-following controller."""

from hexapod_perception.follow_control import compute_cmd, FollowGains
import pytest

G = FollowGains()


def test_at_target_distance_straight_ahead_is_stationary():
    assert compute_cmd(G.target_dist, 0.0, G) == (0.0, 0.0)


def test_inside_deadbands_is_stationary():
    assert compute_cmd(G.target_dist + 0.05, 0.03, G) == (0.0, 0.0)


def test_far_target_drives_forward_with_saturation():
    lin, ang = compute_cmd(5.0, 0.0, G)
    assert lin == pytest.approx(G.max_linear)
    assert ang == 0.0


def test_moderate_distance_is_proportional():
    lin, _ = compute_cmd(1.5, 0.0, G)
    assert lin == pytest.approx(G.kp_linear * (1.5 - G.target_dist - G.dist_deadband))


def test_too_close_backs_up():
    lin, _ = compute_cmd(0.5, 0.0, G)
    assert lin < 0


def test_target_on_left_turns_left():
    _, ang = compute_cmd(2.0, 1.0, G)
    assert ang > 0


def test_target_on_right_turns_right_and_saturates():
    _, ang = compute_cmd(0.5, -2.0, G)
    assert ang == pytest.approx(-G.max_angular)
