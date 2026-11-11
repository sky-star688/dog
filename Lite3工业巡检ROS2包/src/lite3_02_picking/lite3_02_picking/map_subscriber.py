"""Map subscriber for Dog 2 – receive + verify Dog 1's FROZEN shared map.

Dog 2 does NOT build maps.  It refuses to localise against a live (unfrozen)
SLAM map because continuous updates would drift under its feet.

Flow:
  1. Subscribe /shared_map (OccupancyGrid) and /shared_map/metadata (JSON)
  2. Wait for metadata frozen=true with sha256
  3. Verify the received map data hash matches metadata
  4. On match -> republish to local /map for Nav2/AMCL
"""

import hashlib
import json
import time
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


class MapSubscriber(Node):
    def __init__(self):
        super().__init__("map_subscriber_dog2")

        self.declare_parameter("shared_map_topic", "/shared_map")
        self.declare_parameter("local_map_topic", "/map")
        self.declare_parameter("metadata_topic", "/shared_map/metadata")
        self.declare_parameter("map_status_topic", "/dog2/map_status")
        self.declare_parameter("republish_map", True)

        self._latest_shared: Optional[OccupancyGrid] = None
        self._expected_sha = ""
        self._map_frozen = False
        self._map_verified = False
        self._version = 0
        self._mismatch_count = 0
        self._last_map_time = 0.0
        self._last_accept_time = 0.0

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("shared_map_topic").value),
            self._on_shared_map,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("metadata_topic").value),
            self._on_metadata,
            10,
        )

        self._local_pub = None
        if bool(self.get_parameter("republish_map").value):
            self._local_pub = self.create_publisher(
                OccupancyGrid, str(self.get_parameter("local_map_topic").value), 10
            )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("map_status_topic").value), 10
        )
        self.create_timer(2.0, self._publish_status)
        self.get_logger().info(
            "Dog 2 map subscriber ready; waiting for FROZEN %s"
            % str(self.get_parameter("shared_map_topic").value)
        )

    def _on_shared_map(self, message: OccupancyGrid) -> None:
        self._latest_shared = message
        self._last_map_time = time.time()
        if not self._expected_sha:
            return
        actual = hashlib.sha256(bytes(message.data)).hexdigest()
        if actual == self._expected_sha:
            if not self._map_verified:
                self.get_logger().info(
                    "Map VERIFIED v%d sha256=%s %dx%d @ %.3f m/px"
                    % (
                        self._version,
                        actual[:16],
                        message.info.width,
                        message.info.height,
                        message.info.resolution,
                    )
                )
            self._map_verified = True
            self._mismatch_count = 0
            self._last_accept_time = time.time()
            if self._local_pub is not None:
                self._local_pub.publish(message)
        else:
            self._mismatch_count += 1
            if self._mismatch_count <= 3:
                self.get_logger().warn(
                    "Map HASH MISMATCH (count=%d): expected %s got %s"
                    % (self._mismatch_count, self._expected_sha[:16], actual[:16])
                )
            self._map_verified = False

    def _on_metadata(self, message: String) -> None:
        try:
            meta = json.loads(message.data)
        except Exception:
            return
        frozen = bool(meta.get("frozen", False))
        sha = str(meta.get("sha256", ""))
        version = int(meta.get("version", 0))
        if frozen and sha:
            if sha != self._expected_sha:
                self.get_logger().info(
                    "Dog 1 map frozen v%d sha256=%s; awaiting data"
                    % (version, sha[:16])
                )
                self._expected_sha = sha
                self._version = version
                self._map_frozen = True
                self._map_verified = False
                self._mismatch_count = 0
        elif not frozen:
            if self._map_frozen:
                self.get_logger().warn("Dog 1 map UNFROZEN – was v%d" % self._version)
            self._map_frozen = False

    def _publish_status(self) -> None:
        now = time.time()
        msg = String()
        msg.data = json.dumps(
            {
                "map_ready": self._map_verified,
                "map_frozen": self._map_frozen,
                "map_verified": self._map_verified,
                "version": self._version,
                "expected_sha256": (
                    self._expected_sha[:16] if self._expected_sha else ""
                ),
                "hash_mismatch_count": self._mismatch_count,
                "last_map_age_sec": (
                    0.0 if not self._last_map_time else max(0.0, now - self._last_map_time)
                ),
                "last_accepted_age_sec": (
                    0.0
                    if not self._last_accept_time
                    else max(0.0, now - self._last_accept_time)
                ),
                "timestamp": now,
            }
        )
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
