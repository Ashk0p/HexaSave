"""Unit tests for the Pi camera helpers and the robot launch file."""

import importlib.util
import os

import cv2
from hexapod_perception.pi_camera import build_rpicam_command, yuv420_to_bgr
from launch_ros.actions import Node
import numpy as np

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'hexapod_robot.launch.py')


def test_command_streams_raw_yuv_to_stdout_with_full_fov_mode():
    cmd = build_rpicam_command(640, 480, 10.0, '1640:1232:10', 0, False, False)
    assert cmd[0] == 'rpicam-vid'
    assert cmd[cmd.index('--codec') + 1] == 'yuv420'
    assert cmd[cmd.index('--output') + 1] == '-'
    assert cmd[cmd.index('--mode') + 1] == '1640:1232:10'
    assert (cmd[cmd.index('--width') + 1], cmd[cmd.index('--height') + 1]) == ('640', '480')
    assert '--hflip' not in cmd and '--vflip' not in cmd


def test_command_flips_and_no_mode():
    cmd = build_rpicam_command(640, 480, 5.0, '', 1, True, True)
    assert '--mode' not in cmd
    assert '--hflip' in cmd and '--vflip' in cmd
    assert cmd[cmd.index('--camera') + 1] == '1'


def test_yuv420_roundtrip():
    bgr = np.zeros((48, 64, 3), np.uint8)
    bgr[:, :32] = (0, 0, 255)   # red left half
    bgr[:, 32:] = (255, 0, 0)   # blue right half
    i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).tobytes()
    assert len(i420) == 64 * 48 * 3 // 2
    out = yuv420_to_bgr(i420, 64, 48)
    assert out.shape == (48, 64, 3)
    assert np.abs(out.astype(int) - bgr.astype(int)).mean() < 10


def load_launch():
    spec = importlib.util.spec_from_file_location('hexapod_robot_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_starts_full_pipeline():
    nodes = [e for e in load_launch().entities if isinstance(e, Node)]
    names = sorted((n.node_package, n.node_executable) for n in nodes)
    assert names == sorted([
        ('hexapod_perception', 'pi_camera'),
        ('hexapod_perception', 'person_detector'),
        ('hexapod_perception', 'person_follower'),
        ('hexapod_controller', 'gait_controller'),
        ('hexapod_controller', 'servo_bridge'),
        ('hexapod_perception', 'review_recorder'),
    ])
    conditional = {n.node_executable for n in nodes if n.condition is not None}
    assert conditional == {'servo_bridge', 'review_recorder'}
