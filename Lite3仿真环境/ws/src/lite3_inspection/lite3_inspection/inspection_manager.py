import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class InspectionManager(Node):
    """Safety-oriented task coordinator; navigation remains owned by Nav2."""

    def __init__(self):
        super().__init__("inspection_manager")
        self.declare_parameter("detections_topic", "/inspection/detections")
        self.declare_parameter("event_topic", "/inspection/event")
        self.declare_parameter("arm_command_topic", "/arm/command")
        self.declare_parameter("motion_enable_topic", "/inspection/motion_enable")
        self.declare_parameter("target_classes", ["anomaly", "block"])
        self.declare_parameter("minimum_confidence", 0.65)
        self.declare_parameter("stable_frames", 5)
        self.declare_parameter("detection_reset_sec", 1.0)
        self.declare_parameter("auto_pick", False)

        self.running = False
        self.pending_pick = False
        self.stable_count = 0
        self.last_target_time = 0.0
        self.last_detection = None

        self.event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10
        )
        self.arm_pub = self.create_publisher(
            String, str(self.get_parameter("arm_command_topic").value), 10
        )
        self.motion_pub = self.create_publisher(
            Bool, str(self.get_parameter("motion_enable_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self.on_detections,
            10,
        )
        self.create_service(Trigger, "/inspection/start", self.on_start)
        self.create_service(Trigger, "/inspection/stop", self.on_stop)
        self.create_service(Trigger, "/inspection/confirm_pick", self.on_confirm_pick)
        self.create_service(Trigger, "/inspection/place", self.on_place)

    def emit_event(self, event, detail=None):
        payload = {"event": event, "time": time.time(), "detail": detail or {}}
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(message)
        self.get_logger().info(message.data)

    def set_motion(self, enabled):
        message = Bool()
        message.data = enabled
        self.motion_pub.publish(message)

    def arm_command(self, command):
        message = String()
        message.data = command
        self.arm_pub.publish(message)

    def on_start(self, request, response):
        del request
        self.running = True
        self.pending_pick = False
        self.stable_count = 0
        self.set_motion(True)
        self.emit_event("inspection_started")
        response.success = True
        response.message = "inspection monitoring started; motion guard enabled"
        return response

    def on_stop(self, request, response):
        del request
        self.running = False
        self.pending_pick = False
        self.set_motion(False)
        self.arm_command("STOP")
        self.emit_event("inspection_stopped")
        response.success = True
        response.message = "motion and arm stop requested"
        return response

    def on_confirm_pick(self, request, response):
        del request
        if not self.pending_pick:
            response.success = False
            response.message = "no stable target is waiting for confirmation"
            return response
        self.arm_command("PICK")
        self.pending_pick = False
        self.emit_event("pick_requested", self.last_detection)
        response.success = True
        response.message = "PICK command requested"
        return response

    def on_place(self, request, response):
        del request
        self.set_motion(False)
        self.arm_command("PLACE")
        self.emit_event("place_requested")
        response.success = True
        response.message = "PLACE command requested"
        return response

    def on_detections(self, message):
        if not self.running or self.pending_pick:
            return
        try:
            payload = json.loads(message.data)
            targets = set(str(value) for value in self.get_parameter("target_classes").value)
            confidence = float(self.get_parameter("minimum_confidence").value)
            matches = [
                item
                for item in payload.get("detections", [])
                if item.get("class_name") in targets
                and float(item.get("confidence", 0.0)) >= confidence
            ]
        except Exception as exc:
            self.get_logger().error("Invalid detection JSON: %s" % exc)
            return

        now = time.monotonic()
        if matches:
            if now - self.last_target_time > float(
                self.get_parameter("detection_reset_sec").value
            ):
                self.stable_count = 0
            self.last_target_time = now
            self.stable_count += 1
            self.last_detection = max(matches, key=lambda item: item["confidence"])
        else:
            if now - self.last_target_time > float(
                self.get_parameter("detection_reset_sec").value
            ):
                self.stable_count = 0
            return

        if self.stable_count < int(self.get_parameter("stable_frames").value):
            return
        self.set_motion(False)
        self.pending_pick = True
        self.emit_event("stable_target_detected", self.last_detection)
        if bool(self.get_parameter("auto_pick").value):
            self.arm_command("PICK")
            self.pending_pick = False
            self.emit_event("pick_requested_automatically", self.last_detection)
        else:
            self.get_logger().warn(
                "Target found. Inspect alignment, then call /inspection/confirm_pick"
            )


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_motion(False)
        node.arm_command("STOP")
        node.destroy_node()
        rclpy.shutdown()
