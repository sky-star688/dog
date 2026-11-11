"""Publish a fail-closed localization readiness signal.

Reports True only when odometry is fresh AND the configured map -> base
transform exists.  Managers refuse to start or resume autonomous motion unless
this signal is true.
"""

import json
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__("localization_monitor")

        self.declare_parameter("odom_topic", "/leg_odom2")
        self.declare_parameter("status_topic", "/inspection/localization_status")
        self.declare_parameter("info_topic", "/inspection/localization_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("require_tf", True)
        self.declare_parameter("publish_hz", 2.0)

        self._last_odom_time = 0.0
        self._last_odom_frame = ""
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(
            Bool, str(self.get_parameter("status_topic").value), 10
        )
        self._info_pub = self.create_publisher(
            String, str(self.get_parameter("info_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._publish)

    def _on_odom(self, message):
        self._last_odom_time = time.monotonic()
        self._last_odom_frame = message.header.frame_id

    def _publish(self):
        age = (
            time.monotonic() - self._last_odom_time
            if self._last_odom_time
            else None
        )
        odom_ok = bool(self._last_odom_time) and age <= float(
            self.get_parameter("odom_timeout_sec").value
        )
        tf_ok = not bool(self.get_parameter("require_tf").value)
        if bool(self.get_parameter("require_tf").value):
            try:
                tf_ok = self._tf_buffer.can_transform(
                    str(self.get_parameter("map_frame").value),
                    str(self.get_parameter("base_frame").value),
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            except Exception:
                tf_ok = False
        ready = odom_ok and tf_ok

        status = Bool()
        status.data = ready
        self._status_pub.publish(status)

        info = String()
        info.data = json.dumps(
            {
                "ready": ready,
                "odom_fresh": odom_ok,
                "odom_age_sec": None if age is None else round(age, 3),
                "odom_frame": self._last_odom_frame,
                "tf_ok": tf_ok,
                "map_frame": str(self.get_parameter("map_frame").value),
                "base_frame": str(self.get_parameter("base_frame").value),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self._info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
