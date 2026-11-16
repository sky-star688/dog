"""Fleet client model for a single robot (inspector or operator).

ROS-independent.  The ROS nodes adapt this model to publish heartbeats and
target reports, and to receive handoff status from fleet_manager.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Optional


class ClientState(str, Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    TARGET_FOUND = "target_found"
    WAITING_HANDOFF = "waiting_handoff"
    APPROACHING = "approaching"
    POSITIONING = "positioning"
    GRASPING = "grasping"
    PLACING = "placing"
    RETREATING = "retreating"
    RESUMING = "resuming"
    STOPPED = "stopped"
    EMERGENCY = "emergency"
    ERROR = "error"


@dataclass
class FleetClientModel:
    robot_id: str
    robot_role: str = "inspector"  # "inspector" | "operator"
    fleet_id: str = "fleet_01"

    state: ClientState = ClientState.IDLE
    heartbeat_interval_sec: float = 1.0
    last_heartbeat_time: float = 0.0

    pending_target_id: Optional[str] = None
    pending_target_class: Optional[str] = None
    pending_target_confidence: float = 0.0
    pending_target_x: float = 0.0
    pending_target_y: float = 0.0
    pending_target_z: float = 0.0
    stable_frames: int = 0

    active_handoff_id: Optional[str] = None
    handoff_completed: bool = False
    handoff_success: bool = False

    fleet_estop: bool = False
    fleet_command: str = ""

    mission_id: str = ""
    localization_ok: bool = False
    arm_ready: bool = False
    arm_busy: bool = False
    estop: bool = False
    battery_voltage: float = 0.0
    battery_low: bool = False
    waypoint_index: int = -1
    waypoint_total: int = 0
    current_workspace: str = ""

    on_send_heartbeat: Optional[Callable[[Dict[str, Any]], None]] = None
    on_send_target: Optional[Callable[[Dict[str, Any]], None]] = None
    on_handoff_complete: Optional[Callable[[str, bool], None]] = None
    on_fleet_estop: Optional[Callable[[], None]] = None
    on_fleet_command: Optional[Callable[[str], None]] = None

    def heartbeat_payload(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "robot_role": self.robot_role,
            "state": self.state.value,
            "mission_id": self.mission_id,
            "waypoint_index": self.waypoint_index,
            "waypoint_total": self.waypoint_total,
            "localization_ok": self.localization_ok,
            "arm_ready": self.arm_ready,
            "arm_busy": self.arm_busy,
            "estop": self.estop,
            "battery_low": self.battery_low,
            "battery_voltage": self.battery_voltage,
            "current_workspace": self.current_workspace,
            "timestamp": time.time(),
        }

    def target_detection_payload(self) -> Dict[str, Any]:
        return {
            "target_id": self.pending_target_id,
            "inspector_id": self.robot_id,
            "class_name": self.pending_target_class,
            "confidence": self.pending_target_confidence,
            "position": {
                "x": self.pending_target_x,
                "y": self.pending_target_y,
                "z": self.pending_target_z,
            },
            "frame_id": "map",
            "stable_frames": self.stable_frames,
            "requires_operation": self.pending_target_class == "block",
            "timestamp": time.time(),
        }

    def set_target(
        self,
        target_id: str,
        class_name: str,
        confidence: float,
        x: float,
        y: float,
        z: float = 0.0,
        stable_frames: int = 5,
    ) -> None:
        self.pending_target_id = target_id
        self.pending_target_class = class_name
        self.pending_target_confidence = confidence
        self.pending_target_x = x
        self.pending_target_y = y
        self.pending_target_z = z
        self.stable_frames = stable_frames
        self.state = ClientState.TARGET_FOUND

    def clear_target(self) -> None:
        self.pending_target_id = None
        self.pending_target_class = None
        self.pending_target_confidence = 0.0
        self.pending_target_x = 0.0
        self.pending_target_y = 0.0
        self.pending_target_z = 0.0
        self.stable_frames = 0

    def on_handoff_done(self, success: bool) -> None:
        self.handoff_completed = True
        self.handoff_success = success
        self.active_handoff_id = None
        if success:
            self.clear_target()
            self.state = ClientState.RESUMING

    def tick(self, now_sec: Optional[float] = None) -> None:
        now = time.time() if now_sec is None else float(now_sec)
        if now - self.last_heartbeat_time >= self.heartbeat_interval_sec:
            self.last_heartbeat_time = now
            if self.on_send_heartbeat:
                self.on_send_heartbeat(self.heartbeat_payload())

    def report_target(self) -> None:
        if self.pending_target_id is None:
            return
        self.state = ClientState.WAITING_HANDOFF
        if self.on_send_target:
            self.on_send_target(self.target_detection_payload())

    def handle_fleet_message(self, payload: Dict[str, Any]) -> None:
        msg_type = payload.get("type", "")
        if msg_type in ("completed", "handoff_complete"):
            detail = payload.get("detail", {}) or {}
            success = bool(payload.get("success", detail.get("success", False)))
            handoff_id = str(payload.get("handoff_id", detail.get("handoff_id", "")))
            self.on_handoff_done(success)
            if self.on_handoff_complete:
                self.on_handoff_complete(handoff_id, success)
        elif msg_type == "failed":
            detail = payload.get("detail", {}) or {}
            self.on_handoff_done(False)
            if self.on_handoff_complete:
                self.on_handoff_complete(str(detail.get("handoff_id", "")), False)
        elif msg_type == "estop":
            self.fleet_estop = True
            self.state = ClientState.EMERGENCY
            if self.on_fleet_estop:
                self.on_fleet_estop()
        elif msg_type == "command":
            cmd = str(payload.get("command", ""))
            self.fleet_command = cmd
            if self.on_fleet_command:
                self.on_fleet_command(cmd)
