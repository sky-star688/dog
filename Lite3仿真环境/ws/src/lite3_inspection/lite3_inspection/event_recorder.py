"""Record JSON events and optional artifacts into a per-run directory.

Subscribes to the JSON event topic (std_msgs/String) and appends every event
to an fsync-ed JSONL file via lite3_inspection.events.EventStore.
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .events import EventStore


class EventRecorder(Node):
    def __init__(self):
        super().__init__("event_recorder")

        self.declare_parameter("event_topic", "/inspection/event")
        self.declare_parameter("robot_id", "lite3_01")
        self.declare_parameter("mission_id", "runtime")
        self.declare_parameter("root_dir", "")

        root = str(self.get_parameter("root_dir").value)
        if not root:
            root = os.path.join(
                os.path.expanduser("~"),
                ".local",
                "share",
                "lite3_inspection",
                "events",
            )
        robot_id = str(self.get_parameter("robot_id").value)
        mission_id = str(self.get_parameter("mission_id").value)

        self._store = None
        try:
            self._store = EventStore(root, robot_id, mission_id)
            self.get_logger().info(
                "Recording events to %s" % self._store.run_dir
            )
        except Exception as exc:
            self.get_logger().error("Cannot create event store: %s" % exc)

        self.create_subscription(
            String,
            str(self.get_parameter("event_topic").value),
            self._on_event,
            10,
        )

    def _on_event(self, message: String) -> None:
        if self._store is None:
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("event payload must be an object")
        except Exception as exc:
            self.get_logger().error("Invalid event JSON: %s" % exc)
            return
        detail = payload.pop("detail", None)
        event = str(payload.pop("event", "")) or "event"
        fields = dict(payload)
        if detail is not None:
            fields["detail"] = detail
        try:
            self._store.append(event, **fields)
        except Exception as exc:
            self.get_logger().error("Event write failed: %s" % exc)

    def destroy_node(self):
        if self._store is not None:
            try:
                self._store.close("finished")
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EventRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
