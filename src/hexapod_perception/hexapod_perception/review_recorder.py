"""
Record camera frames annotated with the current velocity command for human review.

Writes to <output_dir>/:
  frames/frame_NNNNNN.jpg  annotated frames (detector boxes + velocity panel)
  log.csv                  one row per frame: time, cmd_vel, its age, target
  review.mp4               the annotated frames as a video
  index.html               browsable page of all frames with their velocities
"""

import csv
from datetime import datetime
import html
import os
from typing import Optional, Tuple

import cv2
from geometry_msgs.msg import Twist
from hexapod_perception.image_conversion import imgmsg_to_bgr
from hexapod_perception.review_overlay import draw_overlay
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

CSV_FIELDS = ['frame', 'time_s', 'lin_x', 'ang_z', 'cmd_age_s',
              'target_forward', 'target_left', 'image']


class ReviewRecorder(Node):
    """Subscribe to images, /cmd_vel and the person_target TF; save annotated output."""

    def __init__(self) -> None:
        super().__init__('review_recorder')
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_dir = os.path.join(os.path.expanduser('~'), 'hexapod_review', stamp)
        self.declare_parameter('output_dir', default_dir)
        # Detector output has the boxes drawn; use /camera/image_raw without the detector.
        self.declare_parameter('image_topic', '/perception/yolo_output')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('target_frame', 'person_target')
        self.declare_parameter('target_timeout', 1.0)
        self.declare_parameter('video_fps', 5.0)
        self.declare_parameter('max_lin', 0.3)
        self.declare_parameter('max_ang', 0.6)

        p = self.get_parameter
        self.output_dir = os.path.expanduser(p('output_dir').value)
        self.camera_frame = p('camera_frame').value
        self.target_frame = p('target_frame').value
        self.target_timeout = p('target_timeout').value
        self.video_fps = p('video_fps').value
        self.max_lin = p('max_lin').value
        self.max_ang = p('max_ang').value

        self.frames_dir = os.path.join(self.output_dir, 'frames')
        os.makedirs(self.frames_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.output_dir, 'log.csv'), 'w', newline='')
        self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv.writeheader()
        self.rows: list = []
        self.video: Optional[cv2.VideoWriter] = None
        self.frame_count = 0
        self.start_time = self.get_clock().now()

        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time: Optional[Time] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, p('image_topic').value, self.image_callback, qos)
        self.create_subscription(Twist, p('cmd_topic').value, self.cmd_callback, 10)
        self.get_logger().info(
            f"Recording {p('image_topic').value} + {p('cmd_topic').value} "
            f'to {self.output_dir}')

    def cmd_callback(self, msg: Twist) -> None:
        """Remember the latest velocity command."""
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def latest_target(self, now: Time) -> Optional[Tuple[float, float]]:
        """Return (forward, left) of a fresh person_target TF, if any."""
        try:
            t = self.tf_buffer.lookup_transform(self.camera_frame, self.target_frame, Time())
        except TransformException:
            return None
        age = (now - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if age > self.target_timeout:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def image_callback(self, msg: Image) -> None:
        """Annotate and save one frame."""
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        lin = ang = cmd_age = None
        if self.last_cmd is not None and self.last_cmd_time is not None:
            lin, ang = self.last_cmd.linear.x, self.last_cmd.angular.z
            cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        target = self.latest_target(now)

        annotated = draw_overlay(frame, lin, ang, cmd_age, target, elapsed,
                                 self.max_lin, self.max_ang)
        name = f'frame_{self.frame_count:06d}.jpg'
        cv2.imwrite(os.path.join(self.frames_dir, name), annotated)

        if self.video is None:
            h, w = annotated.shape[:2]
            self.video = cv2.VideoWriter(os.path.join(self.output_dir, 'review.mp4'),
                                         cv2.VideoWriter.fourcc(*'mp4v'),
                                         self.video_fps, (w, h))
        self.video.write(annotated)

        row = {
            'frame': self.frame_count,
            'time_s': f'{elapsed:.3f}',
            'lin_x': '' if lin is None else f'{lin:.3f}',
            'ang_z': '' if ang is None else f'{ang:.3f}',
            'cmd_age_s': '' if cmd_age is None else f'{cmd_age:.2f}',
            'target_forward': '' if target is None else f'{target[0]:.2f}',
            'target_left': '' if target is None else f'{target[1]:.2f}',
            'image': f'frames/{name}',
        }
        self.csv.writerow(row)
        self.csv_file.flush()
        self.rows.append(row)
        self.frame_count += 1

    def finish(self) -> None:
        """Close the video and CSV and write the review page."""
        if self.video is not None:
            self.video.release()
        self.csv_file.close()
        write_index_html(os.path.join(self.output_dir, 'index.html'), self.rows)


def write_index_html(path: str, rows: list) -> None:
    """Write a simple page listing every frame with its velocity command."""
    def cell(value: str) -> str:
        return html.escape(value) if value != '' else '&ndash;'

    body = []
    for r in rows:
        moving = r['lin_x'] not in ('', '0.000') or r['ang_z'] not in ('', '0.000')
        target = ('&ndash;' if r['target_forward'] == '' else
                  f"{r['target_forward']} m fwd, {r['target_left']} m left")
        body.append(
            f"<tr class=\"{'mv' if moving else ''}\"><td>{r['frame']}</td>"
            f"<td>{r['time_s']}</td><td>{cell(r['lin_x'])}</td><td>{cell(r['ang_z'])}</td>"
            f"<td>{cell(r['cmd_age_s'])}</td><td>{target}</td>"
            f"<td><a href=\"{r['image']}\"><img loading=\"lazy\" src=\"{r['image']}\">"
            '</a></td></tr>')
    page = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>Hexapod review</title>
<style>
body{{font:14px system-ui,sans-serif;margin:16px;background:#fafafa;color:#222}}
table{{border-collapse:collapse}} td,th{{padding:4px 8px;border-bottom:1px solid #ddd;
text-align:right;vertical-align:middle}} img{{width:240px;display:block}}
tr.mv td{{background:#eef7ee}} th{{position:sticky;top:0;background:#fafafa}}
</style></head><body>
<h1>Hexapod review</h1>
<p>{len(rows)} frames. Rows with a non-zero velocity command are shaded green.
Video: <a href="review.mp4">review.mp4</a> &middot; data: <a href="log.csv">log.csv</a></p>
<table><tr><th>frame</th><th>t [s]</th><th>lin.x [m/s]</th><th>ang.z [rad/s]</th>
<th>cmd age [s]</th><th>target</th><th>image</th></tr>
{''.join(body)}
</table></body></html>
"""
    with open(path, 'w') as f:
        f.write(page)


def main(args: Optional[list] = None) -> None:
    """Run the review recorder; output is finalised on Ctrl+C."""
    rclpy.init(args=args)
    node = ReviewRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        print(f'Saved {node.frame_count} frames to {node.output_dir}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
