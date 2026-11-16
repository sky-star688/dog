"""Velocity source arbitration for Lite3.

Three independent command sources feed the arbiter; only this node publishes
/inspection/cmd_vel_raw, which cmd_vel_guard then clamps and forwards to
/cmd_vel.  Priority: teleop > aux > nav.  Stale or invalid commands are never
forwarded, and no fresh command means zero velocity.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("nav_topic", "/inspection/cmd_vel_nav")
        self.declare_parameter("teleop_topic", "/inspection/cmd_vel_teleop")
        self.declare_parameter("aux_topic", "/inspection/cmd_vel_aux")
        self.declare_parameter("output_topic", "/inspection/cmd_vel_raw")
        self.declare_parameter("enable_topic", "/inspection/motion_enable")
        self.declare_parameter("estop_topic", "/inspection/estop")
        self.declare_parameter("status_topic", "/inspection/mux_status")
        self.declare_parameter("nav_timeout_sec", 0.30)
        self.declare_parameter("teleop_timeout_sec", 0.30)
        self.declare_parameter("aux_timeout_sec", 0.50)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("start_enabled", False)

        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.estop = False
        self._samples = {}

        self._output_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )

        for source, topic_param in (
            ("nav", "nav_topic"),
            ("teleop", "teleop_topic"),
            ("aux", "aux_topic"),
        ):
            self.create_subscription(
                Twist,
                str(self.get_parameter(topic_param).value),
                lambda msg, source=source: self._on_command(source, msg),
                10,
            )

        self.create_subscription(
            Bool,
            str(self.get_parameter("enable_topic").value),
            self._on_enable,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("estop_topic").value),
            self._on_estop,
            10,
        )

        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            "cmd_vel_mux ready (enabled=%s); teleop > aux > nav" % self.enabled
        )

    def _on_command(self, source, message):
        self._samples[source] = (time.monotonic(), message)

    def _on_enable(self, message):
        self.enabled = bool(message.data)
        if not self.enabled:
            self._output_pub.publish(Twist())

    def _on_estop(self, message):
        was = self.estop
        self.estop = bool(message.data)
        if self.estop and not was:
            self._output_pub.publish(Twist())

    def _timeout_for(self, source):
        return float(self.get_parameter("%s_timeout_sec" % source).value)

    def _publish(self):
        now = time.monotonic()
        if self.estop or not self.enabled:
            self._output_pub.publish(Twist())
            self._publish_status(now, "estop" if self.estop else "disabled")
            return

        for source in ("teleop", "aux", "nav"):
            sample = self._samples.get(source)
            if sample is None:
                continue
            stamp, message = sample
            if now - stamp > self._timeout_for(source):
                continue
            self._output_pub.publish(message)
            self._publish_status(now, source)
            return

        self._output_pub.publish(Twist())
        self._publish_status(now, "timeout")

    def _publish_status(self, now, selected):
        import json

        sources = {}
        for source, (stamp, message) in self._samples.items():
            sources[source] = {
                "age_sec": round(max(0.0, now - stamp), 3),
                "linear_x": message.linear.x,
                "angular_z": message.angular.z,
            }
        message = String()
        message.data = json.dumps(
            {
                "enabled": self.enabled,
                "estop": self.estop,
                "selected": selected,
                "sources": sources,
            },
            sort_keys=True,
        )
        self._status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._output_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
