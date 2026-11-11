"""Localization helper for Dog 2 using Dog 1's frozen shared map.

Dog 2 does NOT run SLAM.  It uses AMCL against the static /shared_map.

This node:
  1. Publishes Dog 2's initial pose to AMCL (/initialpose) after startup
  2. Monitors /amcl_pose covariance for convergence
  3. Publishes /dog2/localization_status (Bool) + /dog2/localization_info
  4. Provides /dog2/set_initial_pose and /dog2/relocalize services

If the initial pose is wrong, AMCL will never converge and picking_manager
will refuse all target assignments.  See config/picking.yaml localization
section for the surveyed home pose of Dog 2 in the shared map.
"""

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class Dog2Localization(Node):
    def __init__(self):
        super().__init__("dog2_localization")

        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("localization_status_topic", "/dog2/localization_status")
        self.declare_parameter("localization_info_topic", "/dog2/localization_info")

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("initial_cov_xy", 0.25)
        self.declare_parameter("initial_cov_yaw", 0.10)

        self.declare_parameter("convergence_cov_xy_max", 0.15)
        self.declare_parameter("convergence_cov_yaw_max", 0.08)
        self.declare_parameter("convergence_stable_frames", 10)
        self.declare_parameter("divergence_cov_xy_min", 1.0)
        self.declare_parameter("auto_relocalize", True)
        self.declare_parameter("relocalize_delay_sec", 5.0)

        self._pose_set = False
        self._latest: Optional[PoseWithCovarianceStamped] = None
        self._localized = False
        self._count = 0
        self._last_lost = 0.0
        self._pose_time = 0.0

        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            10,
        )
        self._status_pub = self.create_publisher(
            Bool, str(self.get_parameter("localization_status_topic").value), 10
        )
        self._info_pub = self.create_publisher(
            String, str(self.get_parameter("localization_info_topic").value), 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self._on_amcl_pose,
            10,
        )
        self.create_service(Trigger, "/dog2/set_initial_pose", self._on_set_initial)
        self.create_service(Trigger, "/dog2/relocalize", self._on_relocalize)
        self.create_timer(1.0, self._tick)
        self.create_timer(3.0, self._auto_set_initial, oneshot=True)

        self.get_logger().info(
            "Dog 2 localisation helper ready; initial pose (%.2f, %.2f, yaw=%.2f)"
            % (
                float(self.get_parameter("initial_x").value),
                float(self.get_parameter("initial_y").value),
                float(self.get_parameter("initial_yaw").value),
            )
        )

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._latest = message
        self._pose_time = time.time()
        cov = message.pose.covariance
        cov_xy = max(cov[0], cov[7])
        cov_yaw = cov[35]
        xy_max = float(self.get_parameter("convergence_cov_xy_max").value)
        yaw_max = float(self.get_parameter("convergence_cov_yaw_max").value)
        stable = int(self.get_parameter("convergence_stable_frames").value)

        if 1e-9 < cov_xy < xy_max and cov_yaw < yaw_max:
            self._count = min(self._count + 1, stable * 2)
        else:
            self._count = max(0, self._count - 1)

        was = self._localized
        self._localized = self._count >= stable
        if self._localized and not was:
            self.get_logger().info(
                "Dog 2 LOCALISED (cov_xy=%.4f yaw=%.4f count=%d)"
                % (cov_xy, cov_yaw, self._count)
            )
        if (
            was
            and cov_xy > float(self.get_parameter("divergence_cov_xy_min").value)
        ):
            self.get_logger().warn(
                "Dog 2 localisation DIVERGED (cov_xy=%.4f)" % cov_xy
            )
            self._last_lost = time.time()

    def _publish_initial_pose(self, inflated: bool = False) -> None:
        x = float(self.get_parameter("initial_x").value)
        y = float(self.get_parameter("initial_y").value)
        yaw = float(self.get_parameter("initial_yaw").value)
        cov_xy = float(self.get_parameter("initial_cov_xy").value)
        cov_yaw = float(self.get_parameter("initial_cov_yaw").value)
        if inflated:
            cov_xy = 1.0
            cov_yaw = 0.5

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        half = yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw

        self._init_pub.publish(msg)
        self._pose_set = True
        self._localized = False
        self._count = 0
        self.get_logger().info(
            "Dog 2 initial pose published (%.2f, %.2f, yaw=%.2f) cov=(%.2f, %.2f)"
            % (x, y, yaw, cov_xy, cov_yaw)
        )

    def _auto_set_initial(self) -> None:
        if not self._pose_set:
            self._publish_initial_pose()

    def _on_set_initial(self, request, response):
        del request
        self._publish_initial_pose()
        response.success = True
        response.message = "initial pose published to AMCL"
        return response

    def _on_relocalize(self, request, response):
        del request
        self.get_logger().info("Manual relocalization requested")
        self._publish_initial_pose(inflated=True)
        response.success = True
        response.message = "global relocalization initialised"
        return response

    def _tick(self) -> None:
        now = time.time()
        if self._pose_time and now - self._pose_time > 3.0:
            self._localized = False
            self._count = 0

        if (
            bool(self.get_parameter("auto_relocalize").value)
            and not self._localized
            and self._last_lost > 0
            and now - self._last_lost
            > float(self.get_parameter("relocalize_delay_sec").value)
        ):
            self.get_logger().warn("Auto-relocalizing after %.0f s lost" % (now - self._last_lost))
            self._publish_initial_pose(inflated=True)
            self._last_lost = 0.0

        status = Bool()
        status.data = self._localized
        self._status_pub.publish(status)

        info = {
            "localized": self._localized,
            "pose_set": self._pose_set,
            "convergence_count": self._count,
            "pose_age_sec": (
                0.0 if not self._pose_time else max(0.0, now - self._pose_time)
            ),
            "initial_x": float(self.get_parameter("initial_x").value),
            "initial_y": float(self.get_parameter("initial_y").value),
            "initial_yaw": float(self.get_parameter("initial_yaw").value),
        }
        if self._latest is not None:
            cov = self._latest.pose.covariance
            info["cov_xy"] = max(cov[0], cov[7])
            info["cov_yaw"] = cov[35]
            info["pose_x"] = self._latest.pose.pose.position.x
            info["pose_y"] = self._latest.pose.pose.position.y
        msg = String()
        msg.data = json.dumps(info)
        self._info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Dog2Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
