"""
Publish Raspberry Pi camera frames on /camera/image_raw using rpicam-vid.

The Pi 5 camera is only reachable through libcamera (cv2.VideoCapture on
/dev/video* does not work), and camera_ros is not installed, so this node
streams raw YUV420 from `rpicam-vid` and converts it with OpenCV.
"""

from collections import deque
import subprocess
import threading
from typing import Deque, List, Optional

import cv2
from hexapod_perception.image_conversion import bgr_to_imgmsg
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def build_rpicam_command(width: int, height: int, fps: float, sensor_mode: str,
                         camera_index: int, hflip: bool, vflip: bool) -> List[str]:
    """Return the rpicam-vid command streaming raw YUV420 frames to stdout."""
    cmd = ['rpicam-vid', '--nopreview', '--timeout', '0',
           '--camera', str(camera_index),
           '--codec', 'yuv420',
           '--width', str(width), '--height', str(height),
           '--framerate', str(fps), '--output', '-']
    if sensor_mode:
        cmd += ['--mode', sensor_mode]
    if hflip:
        cmd.append('--hflip')
    if vflip:
        cmd.append('--vflip')
    return cmd


def yuv420_to_bgr(buf: bytes, width: int, height: int) -> np.ndarray:
    """Convert one packed I420 frame to BGR."""
    yuv = np.frombuffer(buf, dtype=np.uint8).reshape(height * 3 // 2, width)
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)


class PiCamera(Node):
    """Stream the Pi camera into ROS."""

    def __init__(self) -> None:
        super().__init__('pi_camera')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        # Full-sensor binned mode keeps the Camera v2's 62.2 deg field of view;
        # the native 640x480 mode is a centre crop with about half the FOV.
        self.declare_parameter('sensor_mode', '1640:1232:10')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('hflip', False)
        self.declare_parameter('vflip', False)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('topic', '/camera/image_raw')

        p = self.get_parameter
        self.width = p('width').value
        self.height = p('height').value
        self.frame_id = p('frame_id').value
        if self.width % 64 or self.height % 2:
            # rpicam-vid pads rows to 64-pixel strides; packed frames need aligned widths.
            raise ValueError('width must be a multiple of 64 and height even')

        self.pub = self.create_publisher(Image, p('topic').value, 1)
        cmd = build_rpicam_command(self.width, self.height, p('fps').value,
                                   p('sensor_mode').value, p('camera_index').value,
                                   p('hflip').value, p('vflip').value)
        self.get_logger().info('Starting: ' + ' '.join(cmd))
        self.proc: Optional[subprocess.Popen] = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.frames = 0
        # rpicam-vid logs every frame to stderr; drain it so the pipe never blocks.
        self.stderr_tail: Deque[str] = deque(maxlen=10)
        threading.Thread(target=self._drain_stderr, daemon=True).start()
        self.thread = threading.Thread(target=self._read_frames, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self._report)

    def _drain_stderr(self) -> None:
        assert self.proc is not None and self.proc.stderr is not None
        for line in self.proc.stderr:
            self.stderr_tail.append(line.decode(errors='replace').rstrip())

    def _read_frames(self) -> None:
        frame_bytes = self.width * self.height * 3 // 2
        assert self.proc is not None and self.proc.stdout is not None
        while rclpy.ok():
            buf = self.proc.stdout.read(frame_bytes)
            if len(buf) < frame_bytes:
                break
            msg = bgr_to_imgmsg(yuv420_to_bgr(buf, self.width, self.height))
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)
            self.frames += 1
        if rclpy.ok():
            self.proc.wait()
            self.get_logger().error(
                'rpicam-vid stopped (is the camera connected / in use by another '
                'process?): ' + ' | '.join(self.stderr_tail))

    def _report(self) -> None:
        if self.frames == 0:
            self.get_logger().warning('No camera frames received yet')
        else:
            self.get_logger().debug(f'{self.frames / 5.0:.1f} FPS')
        self.frames = 0

    def stop(self) -> None:
        """Terminate rpicam-vid."""
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def main(args: Optional[list] = None) -> None:
    """Run the Pi camera node."""
    rclpy.init(args=args)
    node = PiCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
