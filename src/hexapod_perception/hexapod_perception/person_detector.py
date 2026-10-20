"""Detect people with OpenCV DNN (YOLO ONNX) and broadcast their position as a TF."""

import math
import os
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import TransformStamped
from hexapod_perception.detection import (
    Detection, estimate_target, letterbox, parse_yolo_output)
from hexapod_perception.image_conversion import bgr_to_imgmsg, imgmsg_to_bgr
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


class PersonDetector(Node):
    """Run a YOLO ONNX model through cv2.dnn on each camera frame."""

    def __init__(self) -> None:
        super().__init__('person_detector')

        default_model = os.path.join(
            get_package_share_directory('hexapod_perception'), 'models', 'yolo11n.onnx')
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('input_size', 640)  # must match the ONNX export size
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('hfov_deg', 62.2)  # Raspberry Pi Camera v2
        self.declare_parameter('person_height', 1.7)
        self.declare_parameter('camera_height', 0.15)  # lens height above floor [m]
        self.declare_parameter('min_depth', 0.5)
        self.declare_parameter('max_depth', 6.0)
        self.declare_parameter('enable_gui', False)

        p = self.get_parameter
        self.input_size = p('input_size').value
        self.conf_threshold = p('conf_threshold').value
        self.nms_threshold = p('nms_threshold').value
        self.camera_frame = p('camera_frame').value
        self.hfov_rad = math.radians(p('hfov_deg').value)
        self.person_height = p('person_height').value
        self.camera_height = p('camera_height').value
        self.min_depth = p('min_depth').value
        self.max_depth = p('max_depth').value
        self.enable_gui = p('enable_gui').value

        model_path = p('model_path').value
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.get_logger().info(f'Loaded {model_path} (input {self.input_size})')

        # Keep only the newest frame: inference is slower than the camera rate.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, p('image_topic').value, self.image_callback, qos)
        self.image_pub = self.create_publisher(Image, '/perception/yolo_output', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

    def detect(self, image: np.ndarray) -> Optional[Detection]:
        """Return the highest-confidence person in the image, if any."""
        img_h, img_w = image.shape[:2]
        padded, scale = letterbox(image, self.input_size)
        blob = cv2.dnn.blobFromImage(padded, 1.0 / 255.0, swapRB=True)
        self.net.setInput(blob)
        output = self.net.forward()
        dets = parse_yolo_output(output, scale, img_w, img_h,
                                 self.conf_threshold, self.nms_threshold)
        return dets[0] if dets else None

    def image_callback(self, msg: Image) -> None:
        """Detect, broadcast person_target TF and publish an annotated image."""
        try:
            cv_image = imgmsg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        img_h, img_w = cv_image.shape[:2]
        det = self.detect(cv_image)
        target = None
        if det is not None:
            target = estimate_target(
                det, img_w, img_h, self.hfov_rad, self.person_height,
                self.camera_height, self.min_depth, self.max_depth)

        if target is not None:
            t = TransformStamped()
            # Use the capture time so the follower can reject stale targets.
            stamped = msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0
            t.header.stamp = msg.header.stamp if stamped else self.get_clock().now().to_msg()
            t.header.frame_id = self.camera_frame
            t.child_frame_id = 'person_target'
            t.transform.translation.x = float(target.forward)
            t.transform.translation.y = float(target.left)
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

        publish_debug = self.image_pub.get_subscription_count() > 0
        if not (publish_debug or self.enable_gui):
            return

        if det is not None:
            cv2.rectangle(cv_image, (int(det.x1), int(det.y1)), (int(det.x2), int(det.y2)),
                          (0, 255, 0), 2)
            label = f'{det.score:.2f}'
            if target is not None:
                label += f' {target.forward:.2f}m ({target.method})'
            cv2.putText(cv_image, label, (int(det.x1), max(15, int(det.y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if self.enable_gui:
            cv2.imshow('Hexapod Perception', cv_image)
            cv2.waitKey(1)

        if publish_debug:
            annotated_msg = bgr_to_imgmsg(cv_image)
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)


def main(args: Optional[list] = None) -> None:
    """Run the person detector node."""
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
