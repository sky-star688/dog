import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelGuard(Node):
    """Clamp motion commands and stop when commands time out or motion is disabled."""

    def __init__(self):
        super().__init__("cmd_vel_guard")
        self.declare_parameter("input_topic", "/inspection/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("enable_topic", "/inspection/motion_enable")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("max_linear_x", 0.30)
        self.declare_parameter("max_linear_y", 0.15)
        self.declare_parameter("max_angular_z", 0.50)
        self.declare_parameter("start_enabled", False)

        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.last_command = Twist()
        self.last_command_time = 0.0

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self.on_command,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("enable_topic").value),
            self.on_enable,
            10,
        )
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / publish_hz, self.publish_safe_command)
        self.get_logger().warn(
            "Motion guard started DISABLED" if not self.enabled else "Motion guard started ENABLED"
        )

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def on_command(self, message):
        safe = Twist()
        safe.linear.x = self.clamp(
            message.linear.x, float(self.get_parameter("max_linear_x").value)
        )
        safe.linear.y = self.clamp(
            message.linear.y, float(self.get_parameter("max_linear_y").value)
        )
        safe.angular.z = self.clamp(
            message.angular.z, float(self.get_parameter("max_angular_z").value)
        )
        self.last_command = safe
        self.last_command_time = time.monotonic()

    def on_enable(self, message):
        self.enabled = bool(message.data)
        if not self.enabled:
            self.publisher.publish(Twist())
            self.get_logger().warn("Motion disabled; zero velocity sent")
        else:
            self.get_logger().warn("Motion enabled")

    def publish_safe_command(self):
        age = time.monotonic() - self.last_command_time
        timeout = float(self.get_parameter("timeout_sec").value)
        if self.enabled and age <= timeout:
            self.publisher.publish(self.last_command)
        else:
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
