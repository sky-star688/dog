"""SLAM mapping lifecycle manager for Dog 1.

SLAM itself is computed by slam_toolbox (or another SLAM node) publishing
/map.  This node monitors map availability, provides save/status services and
publishes Dog 1's own map status.
"""

import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MappingManager(Node):
    def __init__(self):
        super().__init__("mapping_manager_dog1")

        self.declare_parameter("robot_id", "lite3_01")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_status_topic", "/dog1/map_status")
        self.declare_parameter("map_save_dir", "")
        self.declare_parameter("status_interval_sec", 2.0)

        self.robot_id = str(self.get_parameter("robot_id").value)
        save_dir = str(self.get_parameter("map_save_dir").value)
        self.map_save_dir = save_dir or os.path.join(
            get_package_share_directory("lite3_01_mapping"), "maps"
        )

        self._latest: OccupancyGrid = None
        self._update_time = 0.0

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            10,
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("map_status_topic").value), 10
        )
        self.create_service(Trigger, "/dog1/save_map", self._on_save_map)
        self.create_service(Trigger, "/dog1/map_status", self._on_map_status)
        self.create_timer(
            float(self.get_parameter("status_interval_sec").value),
            self._publish_status,
        )
        self.get_logger().info("Dog 1 mapping manager ready")

    def _on_map(self, message: OccupancyGrid) -> None:
        self._latest = message
        self._update_time = time.time()

    def _publish_status(self) -> None:
        ready = self._latest is not None
        msg = String()
        msg.data = json.dumps(
            {
                "map_ready": ready,
                "resolution_m": self._latest.info.resolution if ready else 0.0,
                "width": self._latest.info.width if ready else 0,
                "height": self._latest.info.height if ready else 0,
                "age_sec": (
                    0.0 if not self._update_time
                    else max(0.0, time.time() - self._update_time)
                ),
            }
        )
        self.status_pub.publish(msg)

    def _on_save_map(self, request, response):
        del request
        if self._latest is None:
            response.success = False
            response.message = "no map available"
            return response
        try:
            os.makedirs(self.map_save_dir, exist_ok=True)
        except Exception as exc:
            response.success = False
            response.message = "cannot create map dir: %s" % exc
            return response
        response.success = True
        response.message = (
            "map save requested; use slam_toolbox's map_saver to persist to %s"
            % self.map_save_dir
        )
        return response

    def _on_map_status(self, request, response):
        del request
        response.success = self._latest is not None
        response.message = json.dumps(
            {
                "map_ready": self._latest is not None,
                "resolution_m": self._latest.info.resolution if self._latest else 0.0,
                "width": self._latest.info.width if self._latest else 0,
                "height": self._latest.info.height if self._latest else 0,
            }
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MappingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
