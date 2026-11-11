"""Patrol manager for Dog 1 – waypoint patrol, perception, fleet reporting.

Dog 1:
  - Loads patrol waypoints (config/patrol_mission.yaml)
  - Navigates with Nav2 NavigateToPose
  - Runs perception at inspect waypoints (detections_3d JSON)
  - When a stable "block" target is found: pause, report to fleet_manager
  - Waits for handoff completion, then resumes patrol
"""

import json
import os
import time
import uuid

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from lite3_inspection.fleet_client import ClientState, FleetClientModel
from lite3_inspection.geometry import quaternion_from_yaw
from lite3_inspection.robots import RobotContext

from .patrol_state import PatrolModel, PatrolState, PatrolWaypoint


class PatrolManager(Node):
    def __init__(self):
        super().__init__("patrol_manager_dog1")

        self.declare_parameter("robot_id", "lite3_01")
        self.declare_parameter("namespace", "/")
        self.declare_parameter("site_id", "default")
        self.declare_parameter("map_id", "lite3_shared")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("navigation_action", "navigate_to_pose")
        self.declare_parameter("nav_server_wait_sec", 2.0)
        self.declare_parameter("detections_topic", "/dog1/detections_3d")
        self.declare_parameter("localization_topic", "/dog1/localization_status")
        self.declare_parameter("estop_topic", "/dog1/estop")
        self.declare_parameter("motion_enable_topic", "/dog1/motion_enable")
        self.declare_parameter("target_classes", ["anomaly", "block"])
        self.declare_parameter("minimum_confidence", 0.65)
        self.declare_parameter("stable_frames", 5)
        self.declare_parameter("detection_reset_sec", 1.0)
        self.declare_parameter("fleet_mode", True)
        self.declare_parameter("fleet_heartbeat_topic_base", "/fleet/heartbeat")
        self.declare_parameter("fleet_target_topic", "/fleet/target_detection")
        self.declare_parameter("fleet_status_topic", "/lite3_01/handoff_status")

        self.robot_id = str(self.get_parameter("robot_id").value)
        self.fleet_mode = bool(self.get_parameter("fleet_mode").value)
        self.model = PatrolModel(robot_id=self.robot_id)

        try:
            self.context = RobotContext(
                robot_id=self.robot_id,
                namespace=str(self.get_parameter("namespace").value),
                site_id=str(self.get_parameter("site_id").value),
                map_id=str(self.get_parameter("map_id").value),
            )
        except ValueError as exc:
            self.get_logger().error("Invalid robot context: %s" % exc)

        self.fleet = None
        self._fleet_hb_pub = None
        self._fleet_target_pub = None
        if self.fleet_mode:
            self.fleet = FleetClientModel(robot_id=self.robot_id, robot_role="inspector")
            self._fleet_hb_pub = self.create_publisher(
                String,
                "%s/%s"
                % (
                    str(self.get_parameter("fleet_heartbeat_topic_base").value),
                    self.robot_id,
                ),
                10,
            )
            self._fleet_target_pub = self.create_publisher(
                String, str(self.get_parameter("fleet_target_topic").value), 10
            )
            self.create_subscription(
                String,
                str(self.get_parameter("fleet_status_topic").value),
                self._on_fleet_status,
                10,
            )
            self.create_timer(1.0, self._tick_fleet)

        self.localization_ok = True
        self.estop = False
        self._stable_count = 0
        self._last_detection_time = 0.0
        self._last_detection = None
        self._target_armed = True
        self._nav_ready = False
        self._nav_active = False
        self._nav_handle = None

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigation_action").value),
        )
        self.motion_pub = self.create_publisher(
            Bool, str(self.get_parameter("motion_enable_topic").value), 10
        )

        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("localization_topic").value),
            self._on_localization,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("estop_topic").value),
            self._on_estop,
            10,
        )

        self.create_service(Trigger, "/dog1/start_patrol", self._on_start_patrol)
        self.create_service(Trigger, "/dog1/stop_patrol", self._on_stop_patrol)
        self.create_service(Trigger, "/dog1/pause_patrol", self._on_pause_patrol)
        self.create_service(Trigger, "/dog1/resume_patrol", self._on_resume_patrol)

        self.create_timer(0.5, self._tick)
        self._load_mission()

        self.get_logger().info(
            "Dog 1 patrol manager ready. robot=%s fleet_mode=%s waypoints=%d"
            % (self.robot_id, self.fleet_mode, len(self.model.waypoints))
        )

    # ---- Mission ----

    def _load_mission(self) -> None:
        path = str(self.get_parameter("mission_file").value).strip()
        if not path:
            path = os.path.join(
                get_package_share_directory("lite3_01_mapping"),
                "config",
                "patrol_mission.yaml",
            )
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().error("Mission file not found: %s" % path)
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh)
            mission = data.get("mission", data)
            waypoints = []
            for raw in mission.get("waypoints", []):
                waypoints.append(
                    PatrolWaypoint(
                        name=str(raw["name"]),
                        frame_id=str(raw.get("frame_id", "map")),
                        x=float(raw["x"]),
                        y=float(raw["y"]),
                        z=float(raw.get("z", 0.0)),
                        yaw=float(raw.get("yaw", 0.0)),
                        inspect=bool(raw.get("inspect", True)),
                        settle_sec=float(raw.get("settle_sec", 2.0)),
                        tolerance_xy=float(raw.get("tolerance_xy", 0.30)),
                        timeout_sec=float(raw.get("timeout_sec", 120.0)),
                    )
                )
            self.model.set_waypoints(waypoints)
            self.get_logger().info("Loaded %d patrol waypoints" % len(waypoints))
        except Exception as exc:
            self.get_logger().error("Failed to load mission: %s" % exc)

    # ---- Services ----

    def _on_start_patrol(self, request, response):
        del request
        if not self.model.waypoints:
            response.success = False
            response.message = "no waypoints loaded"
            return response
        if self.estop:
            response.success = False
            response.message = "estop asserted"
            return response
        if not self.localization_ok:
            response.success = False
            response.message = "localization not ready"
            return response
        self._ensure_nav_server()
        if not self._nav_ready:
            response.success = False
            response.message = "Nav2 not available"
            return response
        self.model.state = PatrolState.PATROLLING
        self.model.started_at = time.monotonic()
        self.model.current_index = 0
        self.set_motion(True)
        self._navigate_current()
        response.success = True
        response.message = "patrol started"
        return response

    def _on_stop_patrol(self, request, response):
        del request
        self._cancel_nav()
        self.set_motion(False)
        self.model.clear_target()
        self.model.state = PatrolState.STOPPED
        response.success = True
        response.message = "patrol stopped"
        return response

    def _on_pause_patrol(self, request, response):
        del request
        if self.model.state not in (
            PatrolState.PATROLLING,
            PatrolState.SETTLING,
            PatrolState.INSPECTING,
        ):
            response.success = False
            response.message = "not patrolling"
            return response
        self._cancel_nav()
        self.set_motion(False)
        self.model.state = PatrolState.IDLE
        response.success = True
        response.message = "patrol paused"
        return response

    def _on_resume_patrol(self, request, response):
        del request
        if self.model.state != PatrolState.IDLE:
            response.success = False
            response.message = "not paused"
            return response
        if self.estop:
            response.success = False
            response.message = "estop asserted"
            return response
        self._ensure_nav_server()
        self.model.state = PatrolState.PATROLLING
        self.set_motion(True)
        self._navigate_current()
        response.success = True
        response.message = "patrol resumed"
        return response

    # ---- Navigation ----

    def _ensure_nav_server(self) -> None:
        if self._nav_ready:
            return
        self._nav_ready = self._nav_client.wait_for_server(
            timeout_sec=float(self.get_parameter("nav_server_wait_sec").value)
        )
        if self._nav_ready:
            self.get_logger().info("Nav2 NavigateToPose available")

    def _navigate_current(self) -> None:
        waypoint = self.model.current_waypoint
        if waypoint is None:
            self.get_logger().info("Patrol complete – all waypoints visited")
            self.model.state = PatrolState.IDLE
            self.set_motion(False)
            return
        self._ensure_nav_server()
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = waypoint.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = waypoint.z
        qx, qy, qz, qw = quaternion_from_yaw(waypoint.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        self._nav_active = True
        self.model.waypoint_started_at = time.monotonic()
        self.get_logger().info(
            "Navigating to %s (%.2f, %.2f)" % (waypoint.name, waypoint.x, waypoint.y)
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_response)

    def _on_nav_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error("Nav goal failed: %s" % exc)
            self._nav_active = False
            self._advance()
            return
        if not handle.accepted:
            self.get_logger().error("Nav goal rejected")
            self._nav_active = False
            self._advance()
            return
        self._nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        self._nav_active = False
        self._nav_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error("Nav result error: %s" % exc)
            self._advance()
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            waypoint = self.model.current_waypoint
            self.model.waypoints_visited += 1
            if waypoint is not None and waypoint.inspect:
                self.model.state = PatrolState.SETTLING
                self.model.settle_until = time.monotonic() + waypoint.settle_sec
                self.get_logger().info(
                    "Arrived at %s; settling %.1f s"
                    % (waypoint.name, waypoint.settle_sec)
                )
            else:
                self._advance()
        else:
            self.get_logger().warn("Nav failed with status %d" % status)
            self._advance()

    def _cancel_nav(self) -> None:
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_active = False
            self._nav_handle = None

    def _advance(self) -> None:
        self.model.current_index += 1
        if self.model.patrol_complete:
            self.get_logger().info("All waypoints complete. Patrol finished.")
            self.model.state = PatrolState.IDLE
            self.set_motion(False)
            return
        self.model.state = PatrolState.PATROLLING
        self.model.clear_target()
        self._target_armed = True
        self._stable_count = 0
        self._navigate_current()

    # ---- Perception ----

    def _on_detections(self, message: String) -> None:
        if self.model.state not in (PatrolState.SETTLING, PatrolState.INSPECTING):
            return
        if self.model.pending_target:
            return
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
        except Exception:
            return
        targets = set(str(v) for v in self.get_parameter("target_classes").value)
        confidence_min = float(self.get_parameter("minimum_confidence").value)
        matches = [
            d
            for d in detections
            if isinstance(d, dict)
            and d.get("class_name", "") in targets
            and float(d.get("confidence", 0.0)) >= confidence_min
        ]
        now = time.monotonic()
        reset_sec = float(self.get_parameter("detection_reset_sec").value)
        if matches:
            if not self._target_armed:
                return
            if now - self._last_detection_time > reset_sec:
                self._stable_count = 0
            self._last_detection_time = now
            self._stable_count += 1
            self._last_detection = max(
                matches, key=lambda d: float(d.get("confidence", 0.0))
            )
        else:
            if now - self._last_detection_time > reset_sec:
                self._stable_count = 0
                self._target_armed = True
            return

        if self._stable_count < int(self.get_parameter("stable_frames").value):
            return

        self._target_armed = False
        detection = self._last_detection or {}
        class_name = str(detection.get("class_name", "unknown"))
        confidence = float(detection.get("confidence", 0.0))
        point = detection.get("point_map") or [0.0, 0.0, 0.0]
        x = float(point[0]) if len(point) >= 1 else 0.0
        y = float(point[1]) if len(point) >= 2 else 0.0
        z = float(point[2]) if len(point) >= 3 else 0.0

        self.model.set_target(class_name, confidence, x, y, z)
        self.model.targets_detected += 1
        self._cancel_nav()
        self.set_motion(False)
        self.get_logger().info(
            "Dog 1 detected target: %s conf=%.2f at (%.2f, %.2f)"
            % (class_name, confidence, x, y)
        )

        if self.fleet_mode and self.fleet is not None and class_name == "block":
            target = self.model.current_target
            self.fleet.set_target(
                target.target_id, class_name, confidence, x, y, z, self._stable_count
            )
            self.fleet.state = ClientState.TARGET_FOUND
            self.fleet.report_target()
            self.model.state = PatrolState.WAITING_HANDOFF
            self.model.targets_reported += 1
            self.get_logger().info(
                "Target %s reported to fleet_manager; waiting for operator"
                % target.target_id
            )
        else:
            self.get_logger().warn(
                "Target found but fleet_mode=%s class=%s – continuing"
                % (self.fleet_mode, class_name)
            )
            self.model.clear_target()
            self.model.state = PatrolState.PATROLLING
            self.set_motion(True)
            self._advance()

    def _on_localization(self, message: Bool) -> None:
        was = self.localization_ok
        self.localization_ok = bool(message.data)
        if not self.localization_ok and was and self.model.is_active:
            self._cancel_nav()
            self.set_motion(False)
            self.get_logger().error("Localization lost during patrol")

    def _on_estop(self, message: Bool) -> None:
        was = self.estop
        self.estop = bool(message.data)
        if self.estop and not was:
            self._cancel_nav()
            self.set_motion(False)
            self.model.state = PatrolState.EMERGENCY
            self.get_logger().error("Dog 1 ESTOP asserted")

    # ---- Fleet ----

    def _tick_fleet(self) -> None:
        if self.fleet is None:
            return
        self.fleet.localization_ok = self.localization_ok
        self.fleet.estop = self.estop
        self.fleet.waypoint_index = self.model.current_index
        self.fleet.waypoint_total = len(self.model.waypoints)
        self.fleet.tick()

    def _on_fleet_status(self, message: String) -> None:
        if self.fleet is None:
            return
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        msg_type = str(payload.get("type", ""))
        detail = payload.get("detail", {}) or {}
        if msg_type in ("completed", "failed"):
            success = bool(payload.get("success", msg_type == "completed"))
            self.get_logger().info(
                "Fleet: handoff %s (success=%s)"
                % ("completed" if success else "failed", success)
            )
            self.model.handoff_complete = True
            self.model.handoff_success = success
            self.model.clear_target()
            self._target_armed = True
            self._stable_count = 0
            self.model.state = PatrolState.RESUMING
            if success:
                self.set_motion(True)
                self._advance()
            else:
                self.get_logger().warn("Handoff failed; skipping target and continuing")
                self.set_motion(True)
                self._advance()
        elif msg_type == "requested":
            self.model.handoff_pending = True
            self.model.state = PatrolState.WAITING_HANDOFF

    # ---- Helpers ----

    def set_motion(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.motion_pub.publish(msg)

    def _tick(self) -> None:
        if (
            self.model.state == PatrolState.SETTLING
            and time.monotonic() >= self.model.settle_until
        ):
            self.model.state = PatrolState.INSPECTING
            self.get_logger().info("Settling complete – inspecting")

        if self._nav_active and self.model.current_waypoint is not None:
            elapsed = time.monotonic() - self.model.waypoint_started_at
            if elapsed > self.model.current_waypoint.timeout_sec:
                self._cancel_nav()
                self.get_logger().warn(
                    "Navigation timeout at %s" % self.model.current_waypoint.name
                )
                self._advance()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_motion(False)
        node.destroy_node()
        rclpy.shutdown()
