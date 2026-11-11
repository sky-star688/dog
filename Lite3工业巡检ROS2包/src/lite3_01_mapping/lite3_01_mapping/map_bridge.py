"""Map bridge for Dog 1 → Dog 2 with freeze + SHA256 snapshot.

Dog 1 builds the SLAM map.  Dog 2 localises against a *static* snapshot, so
live SLAM updates must be frozen before Dog 2 starts navigating.

Services:
  /dog1/freeze_map      – freeze current map, compute hash, publish snapshot
  /dog1/unfreeze_map    – resume live relay
  /dog1/map_snapshot    – return snapshot metadata JSON

Topics:
  /shared_map           – OccupancyGrid (live or frozen)
  /shared_map/metadata  – JSON {frozen, version, sha256, resolution, ...}
"""

import hashlib
import json
import time
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MapBridge(Node):
    def __init__(self):
        super().__init__("map_bridge_dog1")

        self.declare_parameter("robot_id", "lite3_01")
        self.declare_parameter("map_in_topic", "/map")
        self.declare_parameter("map_out_topic", "/shared_map")
        self.declare_parameter("metadata_topic", "/shared_map/metadata")
        self.declare_parameter("status_interval_sec", 2.0)
        self.declare_parameter("auto_freeze_after_sec", 0.0)

        self.robot_id = str(self.get_parameter("robot_id").value)
        self.auto_freeze_sec = float(self.get_parameter("auto_freeze_after_sec").value)

        self._latest_map: Optional[OccupancyGrid] = None
        self._frozen_map: Optional[OccupancyGrid] = None
        self._frozen = False
        self._publish_start = time.time()
        self._version = 0
        self._last_map_time = 0.0
        self._last_freeze_sha = ""

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_in_topic").value),
            self._on_map,
            10,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("map_out_topic").value), 10
        )
        self.meta_pub = self.create_publisher(
            String, str(self.get_parameter("metadata_topic").value), 10
        )

        self.create_service(Trigger, "/dog1/freeze_map", self._on_freeze)
        self.create_service(Trigger, "/dog1/unfreeze_map", self._on_unfreeze)
        self.create_service(Trigger, "/dog1/map_snapshot", self._on_snapshot)

        self.create_timer(
            float(self.get_parameter("status_interval_sec").value),
            self._publish_metadata,
        )
        self.get_logger().info(
            "Dog 1 map bridge ready: %s -> %s"
            % (
                str(self.get_parameter("map_in_topic").value),
                str(self.get_parameter("map_out_topic").value),
            )
        )

    @staticmethod
    def _hash(message: OccupancyGrid) -> str:
        return hashlib.sha256(bytes(message.data)).hexdigest()

    def _on_map(self, message: OccupancyGrid) -> None:
        self._latest_map = message
        self._last_map_time = time.time()
        if self.auto_freeze_sec > 0 and not self._frozen:
            if time.time() - self._publish_start >= self.auto_freeze_sec:
                self._do_freeze()
                self.get_logger().info(
                    "Auto-frozen after %.0f s" % self.auto_freeze_sec
                )
        if not self._frozen:
            self.map_pub.publish(message)

    def _do_freeze(self) -> str:
        if self._latest_map is None:
            return ""
        self._frozen_map = self._latest_map
        self._frozen = True
        self._version += 1
        self._last_freeze_sha = self._hash(self._frozen_map)
        self.map_pub.publish(self._frozen_map)
        self.get_logger().info(
            "Map FROZEN v%d sha256=%s %dx%d @ %.3f m/px"
            % (
                self._version,
                self._last_freeze_sha[:16],
                self._frozen_map.info.width,
                self._frozen_map.info.height,
                self._frozen_map.info.resolution,
            )
        )
        return self._last_freeze_sha

    def _on_freeze(self, request, response):
        del request
        if self._latest_map is None:
            response.success = False
            response.message = "no map available to freeze"
            return response
        sha = self._do_freeze()
        response.success = True
        response.message = "map frozen v%d sha256=%s" % (self._version, sha[:16])
        return response

    def _on_unfreeze(self, request, response):
        del request
        if not self._frozen:
            response.success = False
            response.message = "map is not frozen"
            return response
        self._frozen = False
        self._frozen_map = None
        self._publish_start = time.time()
        self.get_logger().info("Map unfrozen – live relay resumed")
        response.success = True
        response.message = "map unfrozen; live relay resumed"
        return response

    def _on_snapshot(self, request, response):
        del request
        if not self._frozen or self._frozen_map is None:
            response.success = False
            response.message = json.dumps({"frozen": False})
            return response
        info = {
            "frozen": True,
            "version": self._version,
            "sha256": self._last_freeze_sha,
            "width": self._frozen_map.info.width,
            "height": self._frozen_map.info.height,
            "resolution_m": self._frozen_map.info.resolution,
            "origin_x": self._frozen_map.info.origin.position.x,
            "origin_y": self._frozen_map.info.origin.position.y,
            "source_robot": self.robot_id,
        }
        response.success = True
        response.message = json.dumps(info)
        return response

    def _publish_metadata(self) -> None:
        frozen = self._frozen and self._frozen_map is not None
        current = self._frozen_map if frozen else self._latest_map
        meta = {
            "map_ready": current is not None,
            "frozen": frozen,
            "version": self._version if frozen else 0,
            "source_robot": self.robot_id,
            "topic": str(self.get_parameter("map_out_topic").value),
            "age_sec": (
                0.0 if not self._last_map_time
                else max(0.0, time.time() - self._last_map_time)
            ),
            "timestamp": time.time(),
        }
        if current is not None:
            meta["sha256"] = (
                self._last_freeze_sha if frozen and self._last_freeze_sha
                else self._hash(current)
            )
            meta["resolution_m"] = current.info.resolution
            meta["width"] = current.info.width
            meta["height"] = current.info.height
            meta["origin_x"] = current.info.origin.position.x
            meta["origin_y"] = current.info.origin.position.y
        msg = String()
        msg.data = json.dumps(meta, ensure_ascii=False)
        self.meta_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
