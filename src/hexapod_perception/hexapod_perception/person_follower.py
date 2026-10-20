"""Turn the person_target TF into /cmd_vel commands."""

from typing import Optional

from geometry_msgs.msg import Twist
from hexapod_perception.follow_control import compute_cmd, FollowGains
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PersonFollower(Node):
    """Proportional follower that stops when the target is lost or stale."""

    def __init__(self) -> None:
        super().__init__('person_follower')

        defaults = FollowGains()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('camera_frame', 'camera_link')
        # Must exceed detector frame period + inference time (~0.7 s at 640 on a Pi 5).
        self.declare_parameter('max_stale_sec', 1.0)

        self.gains = FollowGains(
            **{name: self.get_parameter(name).value for name in vars(defaults)})
        self.camera_frame = self.get_parameter('camera_frame').value
        self.max_stale_sec = self.get_parameter('max_stale_sec').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Person follower running')

    def stop_robot(self) -> None:
        """Publish a zero velocity command."""
        self.cmd_pub.publish(Twist())

    def control_loop(self) -> None:
        """Compute and publish a velocity command from the latest target."""
        cmd = Twist()
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, 'person_target', Time())
        except TransformException:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if age <= self.max_stale_sec:
            cmd.linear.x, cmd.angular.z = compute_cmd(
                t.transform.translation.x, t.transform.translation.y, self.gains)
        self.cmd_pub.publish(cmd)


def main(args: Optional[list] = None) -> None:
    """Run the person follower node."""
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()  # stop on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
