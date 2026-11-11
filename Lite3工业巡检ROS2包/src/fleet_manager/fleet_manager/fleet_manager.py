"""Fleet-level coordinator node for dual-Lite3 collaborative missions.

Responsibilities:
  - Monitor heartbeats from both robots
  - Receive target detections from the inspector (Dog 1)
  - Manage serial handoff lifecycle (inspector → operator → inspector)
  - Workspace reservation and release
  - Fleet-level safety: estop propagation, battery monitoring
  - Fleet-level commands: stop_all, resume_all, return_all, report
"""

import json
import os
import time
import uuid
from typing import Any, Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .fleet_safety import FleetSafetyChecker, FleetSafetyLimits, SafetyDistance
from .fleet_state import FleetModel, FleetState, RobotRole, RobotStatus
from .handoff import HandoffPhase, HandoffState, HandoffTarget
from .workspace import WorkspaceGeometry, WorkspaceManager


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")

        self.declare_parameter("fleet_config", "")
        self.declare_parameter("fleet_id", "fleet_01")
        self.declare_parameter("inspector_id", "lite3_01")
        self.declare_parameter("operator_id", "lite3_02")
        self.declare_parameter("heartbeat_topic_base", "/fleet/heartbeat")
        self.declare_parameter("target_topic", "/fleet/target_detection")
        self.declare_parameter("status_topic", "/fleet/status")
        self.declare_parameter("event_topic", "/fleet/event")
        self.declare_parameter("estop_topic", "/fleet/estop")
        self.declare_parameter("handoff_timeout_sec", 600.0)
        self.declare_parameter("auto_start", True)

        self.fleet_id = str(self.get_parameter("fleet_id").value)
        self.inspector_id = str(self.get_parameter("inspector_id").value)
        self.operator_id = str(self.get_parameter("operator_id").value)

        self.model = FleetModel(state=FleetState.INIT, started_at=time.monotonic())
        self.model.register_robot(self.inspector_id, RobotRole.INSPECTOR)
        self.model.register_robot(self.operator_id, RobotRole.OPERATOR)

        self.workspace_manager = WorkspaceManager()
        self.safety = FleetSafetyChecker(
            distances=SafetyDistance(),
            limits=FleetSafetyLimits(
                max_handoff_duration_sec=float(
                    self.get_parameter("handoff_timeout_sec").value
                )
            ),
        )
        self._active_handoff: Optional[HandoffState] = None
        self._assignment_pubs: Dict[str, Any] = {}
        self._last_emergency = False

        self._load_fleet_config()

        self.event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.estop_pub = self.create_publisher(
            String, str(self.get_parameter("estop_topic").value), 10
        )

        hb_base = str(self.get_parameter("heartbeat_topic_base").value)
        for robot_id in (self.inspector_id, self.operator_id):
            self.create_subscription(
                String,
                "%s/%s" % (hb_base, robot_id),
                lambda msg, rid=robot_id: self._on_heartbeat(msg, rid),
                10,
            )
        self.create_subscription(
            String,
            str(self.get_parameter("target_topic").value),
            self._on_target_detection,
            10,
        )
        # Handoff progress reported by the operator dog.
        self.create_subscription(
            String,
            "/fleet/handoff_status",
            self._on_handoff_status,
            10,
        )

        self.create_service(Trigger, "/fleet/stop_all", self._on_stop_all)
        self.create_service(Trigger, "/fleet/resume_all", self._on_resume_all)
        self.create_service(Trigger, "/fleet/return_all", self._on_return_all)
        self.create_service(Trigger, "/fleet/emergency_stop", self._on_emergency_stop)
        self.create_service(Trigger, "/fleet/generate_report", self._on_generate_report)

        self.create_timer(0.5, self._tick)
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            "Fleet manager '%s' ready. Inspector=%s Operator=%s"
            % (self.fleet_id, self.inspector_id, self.operator_id)
        )
        self._emit_event("fleet_initialised")
        if bool(self.get_parameter("auto_start").value):
            self.model.state = FleetState.READY

    # ---- Config ----

    def _load_fleet_config(self) -> None:
        path = str(self.get_parameter("fleet_config").value).strip()
        if not path:
            path = os.path.join(
                get_package_share_directory("fleet_manager"), "config", "fleet.yaml"
            )
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().warn("Fleet config not found: %s (using defaults)" % path)
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            for ws_data in data.get("workspaces", []):
                geometry = WorkspaceGeometry(
                    workspace_id=str(ws_data["id"]),
                    approach_x=float(ws_data.get("approach_x", 0.0)),
                    approach_y=float(ws_data.get("approach_y", 0.0)),
                    approach_yaw=float(ws_data.get("approach_yaw", 0.0)),
                    centre_x=float(ws_data.get("centre_x", 0.0)),
                    centre_y=float(ws_data.get("centre_y", 0.0)),
                    retreat_x=float(ws_data.get("retreat_x", 0.0)),
                    retreat_y=float(ws_data.get("retreat_y", 0.0)),
                    retreat_yaw=float(ws_data.get("retreat_yaw", 0.0)),
                    boundary=ws_data.get("boundary", []),
                    frame_id=str(ws_data.get("frame_id", "map")),
                    max_occupancy_sec=float(ws_data.get("max_occupancy_sec", 300.0)),
                    reservation_timeout_sec=float(
                        ws_data.get("reservation_timeout_sec", 600.0)
                    ),
                )
                self.workspace_manager.register_workspace(geometry)
            safety_cfg = data.get("safety", {}) or {}
            if safety_cfg:
                self.safety = FleetSafetyChecker(
                    distances=SafetyDistance(
                        robot_to_robot_min_m=float(
                            safety_cfg.get("robot_to_robot_min_m", 3.0)
                        ),
                        inspector_to_workspace_min_m=float(
                            safety_cfg.get("inspector_to_workspace_min_m", 5.0)
                        ),
                        operator_approach_offset_m=float(
                            safety_cfg.get("operator_approach_offset_m", 2.0)
                        ),
                    ),
                    limits=FleetSafetyLimits(
                        max_handoff_duration_sec=float(
                            safety_cfg.get("max_handoff_duration_sec", 600.0)
                        ),
                        max_operator_response_sec=float(
                            safety_cfg.get("max_operator_response_sec", 30.0)
                        ),
                        battery_warning_voltage=float(
                            safety_cfg.get("battery_warning_voltage", 22.0)
                        ),
                        battery_critical_voltage=float(
                            safety_cfg.get("battery_critical_voltage", 21.0)
                        ),
                    ),
                )
            self.get_logger().info(
                "Loaded fleet config: %d workspaces" % len(self.workspace_manager.list_workspaces())
            )
        except Exception as exc:
            self.get_logger().error("Failed to load fleet config: %s" % exc)

    # ---- Heartbeats ----

    def _on_heartbeat(self, message: String, robot_id: str) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("heartbeat must be an object")
        except Exception as exc:
            self.get_logger().error("Invalid heartbeat from %s: %s" % (robot_id, exc))
            return
        record = self.model.robots.get(robot_id)
        if record is None:
            self.get_logger().warn("Heartbeat from unknown robot: %s" % robot_id)
            return
        was_estop = record.estop
        record.update_heartbeat(payload)

        if record.estop and not was_estop:
            self.get_logger().error("ESTOP from %s – stopping fleet" % robot_id)
            self.model.emergency_stop()
            self._emit_event("fleet_emergency_stop", {"triggered_by": robot_id})
            self._publish_estop(robot_id)

        if record.battery_voltage > 0:
            ok, msg, action = self.safety.check_battery(record.battery_voltage, robot_id)
            if not ok:
                self.get_logger().warn(msg)
                self._emit_event(
                    "battery_warning",
                    {"robot_id": robot_id, "voltage": record.battery_voltage, "action": action},
                )

    # ---- Targets ----

    def _on_target_detection(self, message: String) -> None:
        if self.model.state == FleetState.EMERGENCY:
            self.get_logger().warn("Target ignored: fleet in emergency")
            return
        if self._active_handoff is not None and self._active_handoff.is_active:
            self.get_logger().warn("Target ignored: handoff already in progress")
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("target must be an object")
        except Exception as exc:
            self.get_logger().error("Invalid target detection: %s" % exc)
            return

        target = HandoffTarget(
            target_id=str(payload.get("target_id", uuid.uuid4().hex[:12])),
            class_name=str(payload.get("class_name", "unknown")),
            confidence=float(payload.get("confidence", 0.0)),
            coarse_x=float(payload.get("position", {}).get("x", 0.0)),
            coarse_y=float(payload.get("position", {}).get("y", 0.0)),
            coarse_z=float(payload.get("position", {}).get("z", 0.0)),
            coarse_yaw=0.0,
            frame_id=str(payload.get("frame_id", "map")),
            inspector_id=str(payload.get("inspector_id", self.inspector_id)),
            extra={"requires_operation": bool(payload.get("requires_operation", False))},
        )
        errors = target.validate()
        if errors:
            self.get_logger().error("Invalid target: %s" % "; ".join(errors))
            return

        self.get_logger().info(
            "Target detected: %s (%s conf=%.2f) at (%.2f, %.2f)"
            % (target.target_id, target.class_name, target.confidence, target.coarse_x, target.coarse_y)
        )
        self._emit_event("target_detected", target.to_dict())

        if target.class_name == "anomaly" and not target.extra.get("requires_operation"):
            self.model.pending_targets.append(target.to_dict())
            self.get_logger().info("Anomaly recorded (no operation needed)")
            return

        self._initiate_handoff(target)

    def _initiate_handoff(self, target: HandoffTarget) -> None:
        operator = self.model.robots.get(self.operator_id)
        if operator is None or not operator.online:
            self._emit_event("handoff_failed", {"reason": "operator_offline", "target_id": target.target_id})
            self.get_logger().error("Cannot initiate handoff: operator offline")
            return
        if not operator.can_accept_handoff:
            self._emit_event(
                "handoff_failed",
                {
                    "reason": "operator_not_ready",
                    "target_id": target.target_id,
                    "operator_status": operator.status.value,
                },
            )
            self.get_logger().error(
                "Cannot initiate handoff: operator not ready (status=%s)" % operator.status.value
            )
            return

        workspace_id = self._find_free_workspace()
        if workspace_id is None:
            self._emit_event("handoff_failed", {"reason": "no_free_workspace", "target_id": target.target_id})
            self.get_logger().error("No free workspace available")
            return

        handoff = HandoffState.create(target, self.inspector_id, self.operator_id, workspace_id)
        try:
            self.workspace_manager.reserve(workspace_id, self.operator_id, target.target_id)
            self.model.workspace_reservations[workspace_id] = self.operator_id
        except Exception as exc:
            self._emit_event("handoff_failed", {"reason": "workspace_reserve_failed", "error": str(exc)})
            self.get_logger().error("Failed to reserve workspace: %s" % exc)
            return

        self._active_handoff = handoff
        self.model.state = FleetState.HANDOFF_IN_PROGRESS
        self.model.stats["total_targets_detected"] += 1
        self._emit_event("handoff_initiated", handoff.to_dict())
        self.get_logger().info(
            "Handoff %s initiated: %s -> %s, workspace=%s"
            % (handoff.handoff_id, self.inspector_id, self.operator_id, workspace_id)
        )
        self._publish_target_assignment(handoff)
        # Tell the inspector to wait for the operator.
        self._publish_handoff_status(
            {
                "type": "requested",
                "handoff_id": handoff.handoff_id,
                "target_id": target.target_id,
                "from_robot": self.inspector_id,
                "to_robot": self.operator_id,
            },
            robot_id=self.inspector_id,
        )

    def _find_free_workspace(self) -> Optional[str]:
        for ws_id in self.workspace_manager.list_workspaces():
            if self.workspace_manager.is_workspace_free(ws_id):
                return ws_id
        return None

    def _publish_target_assignment(self, handoff: HandoffState) -> None:
        topic = "/%s/target_assignment" % self.operator_id
        publisher = self._assignment_pubs.get(topic)
        if publisher is None:
            publisher = self.create_publisher(String, topic, 10)
            self._assignment_pubs[topic] = publisher
        msg = String()
        msg.data = json.dumps(
            {
                "type": "target_assignment",
                "handoff_id": handoff.handoff_id,
                "target_id": handoff.target.target_id,
                "class_name": handoff.target.class_name,
                "confidence": handoff.target.confidence,
                "coarse_pose": {
                    "x": handoff.target.coarse_x,
                    "y": handoff.target.coarse_y,
                    "z": handoff.target.coarse_z,
                    "yaw": handoff.target.coarse_yaw,
                    "frame_id": handoff.target.frame_id,
                },
                "workspace_id": handoff.workspace_id,
                "inspector_id": handoff.from_robot,
            },
            ensure_ascii=False,
        )
        publisher.publish(msg)
        self.get_logger().info("Published target assignment to %s" % topic)

    # ---- Handoff progress from operator ----

    def _on_handoff_status(self, message: String) -> None:
        if self._active_handoff is None:
            return
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                return
        except Exception:
            return
        if str(payload.get("robot_id", "")) != self.operator_id:
            return
        msg_type = str(payload.get("type", ""))
        detail = payload.get("detail", {}) or {}
        handoff_id = str(detail.get("handoff_id", ""))

        if msg_type == "accepted":
            try:
                self._active_handoff.accept()
                self._emit_event("handoff_accepted", self._active_handoff.to_dict())
            except Exception as exc:
                self.get_logger().error("Handoff accept failed: %s" % exc)
            return

        if handoff_id and handoff_id != self._active_handoff.handoff_id:
            return

        if msg_type == "phase":
            phase = str(detail.get("phase", ""))
            try:
                self._active_handoff.transition_to(HandoffPhase(phase))
                self._emit_event(
                    "handoff_phase",
                    {"handoff_id": handoff_id, "phase": phase},
                )
            except Exception as exc:
                self.get_logger().error("Handoff phase update failed: %s" % exc)
        elif msg_type == "completed":
            self._finish_handoff(success=True, error="")
        elif msg_type == "failed":
            self._finish_handoff(success=False, error=str(detail.get("reason", "operator_failed")))

    def _finish_handoff(self, success: bool, error: str) -> None:
        if self._active_handoff is None:
            return
        handoff = self._active_handoff
        if success:
            handoff.complete()
        else:
            handoff.fail(error)
        ws_id = handoff.workspace_id
        try:
            self.workspace_manager.release(ws_id, self.operator_id)
            self.model.workspace_reservations.pop(ws_id, None)
        except Exception as exc:
            self.get_logger().error("Workspace release failed: %s" % exc)
        if success:
            self.model.stats["total_handoffs_completed"] += 1
            self.model.stats["total_targets_handled"] += 1
        self.model.state = FleetState.MISSION_ACTIVE
        self._emit_event(
            "handoff_completed" if success else "handoff_failed", handoff.to_dict()
        )
        # Notify both dogs so the inspector can resume patrol.
        self._publish_handoff_status(
            {
                "type": "completed" if success else "failed",
                "handoff_id": handoff.handoff_id,
                "success": success,
                "error": error,
            },
            robot_id=self.inspector_id,
        )
        self._publish_handoff_status(
            {
                "type": "completed" if success else "failed",
                "handoff_id": handoff.handoff_id,
                "success": success,
                "error": error,
            },
            robot_id=self.operator_id,
        )
        self._active_handoff = None
        self.get_logger().info(
            "Handoff %s. Inspector may resume patrol."
            % ("completed" if success else "failed")
        )

    def _publish_handoff_status(self, payload: Dict[str, Any], robot_id: str) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        topic = "/%s/handoff_status" % robot_id
        publisher = self._assignment_pubs.get(topic)
        if publisher is None:
            publisher = self.create_publisher(String, topic, 10)
            self._assignment_pubs[topic] = publisher
        publisher.publish(msg)

    # ---- Fleet commands ----

    def _on_stop_all(self, request, response):
        del request
        self.model.state = FleetState.ALL_STOPPED
        if self._active_handoff and self._active_handoff.is_active:
            self._active_handoff.cancel()
            self._active_handoff = None
        self._emit_event("fleet_stop_all")
        self.get_logger().info("Fleet: all robots stopped")
        response.success = True
        response.message = "stop_all command issued"
        return response

    def _on_resume_all(self, request, response):
        del request
        if self.model.state == FleetState.EMERGENCY:
            response.success = False
            response.message = "cannot resume: fleet is in emergency"
            return response
        self.model.state = FleetState.MISSION_ACTIVE
        self._emit_event("fleet_resume_all")
        response.success = True
        response.message = "resume_all command issued"
        return response

    def _on_return_all(self, request, response):
        del request
        self.model.state = FleetState.RETURNING
        if self._active_handoff and self._active_handoff.is_active:
            self._active_handoff.cancel()
            self._active_handoff = None
        self._emit_event("fleet_return_all")
        self.get_logger().info("Fleet: all robots returning to base")
        response.success = True
        response.message = "return_all command issued"
        return response

    def _on_emergency_stop(self, request, response):
        del request
        self.model.emergency_stop()
        if self._active_handoff and self._active_handoff.is_active:
            self._active_handoff.cancel()
            self._active_handoff = None
        self._emit_event("fleet_emergency_stop", {"triggered_by": "operator"})
        self._publish_estop("operator")
        self.get_logger().error("Fleet: EMERGENCY STOP")
        response.success = True
        response.message = "emergency_stop: all robots stopped"
        return response

    def _on_generate_report(self, request, response):
        del request
        report = self._generate_report()
        self._emit_event("fleet_report_generated", {"report": report})
        response.success = True
        response.message = json.dumps(report, ensure_ascii=False, indent=2)
        return response

    def _generate_report(self) -> Dict[str, Any]:
        uptime = max(0.0, time.monotonic() - self.model.started_at)
        handoffs = [h.to_dict() for h in self.model.handoffs.values()]
        return {
            "fleet_id": self.fleet_id,
            "state": self.model.state.value,
            "uptime_sec": uptime,
            "robots": {rid: rec.to_dict() for rid, rec in self.model.robots.items()},
            "stats": self.model.stats,
            "workspaces": self.workspace_manager.to_dict(),
            "handoffs": handoffs,
            "active_handoff": (
                None if self._active_handoff is None else self._active_handoff.to_dict()
            ),
            "pending_targets": len(self.model.pending_targets),
            "inspector_id": self.inspector_id,
            "operator_id": self.operator_id,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }

    # ---- Timers ----

    def _tick(self) -> None:
        now = time.monotonic()
        for ws_id in self.workspace_manager.force_release_expired():
            self._emit_event("workspace_reservation_expired", {"workspace_id": ws_id})
            self.model.workspace_reservations.pop(ws_id, None)

        if self._active_handoff is not None and self._active_handoff.is_active:
            ok, msg = self.safety.check_handoff_timeout(self._active_handoff.elapsed_sec)
            if not ok:
                self.get_logger().error("Handoff timeout: %s" % msg)
                self._finish_handoff(success=False, error=msg)
            elif self._active_handoff.phase.value == "requested":
                ok, msg = self.safety.check_operator_response(self._active_handoff.elapsed_sec)
                if not ok:
                    self.get_logger().error("Operator response timeout: %s" % msg)
                    self._finish_handoff(success=False, error=msg)

        for robot_id, record in self.model.robots.items():
            if record.last_heartbeat > 0 and not record.online:
                self._emit_event(
                    "robot_offline",
                    {"robot_id": robot_id, "age_sec": now - record.last_heartbeat},
                )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._generate_report(), ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(msg)

    def _publish_estop(self, triggered_by: str) -> None:
        msg = String()
        msg.data = json.dumps({"estop": True, "triggered_by": triggered_by})
        self.estop_pub.publish(msg)

    def _emit_event(self, event_type: str, detail: Optional[Dict[str, Any]] = None) -> None:
        payload = {
            "event": event_type,
            "time": time.time(),
            "fleet_id": self.fleet_id,
            "detail": detail or {},
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
