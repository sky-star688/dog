"""Fleet-level state model for dual-dog collaborative missions (ROS-free)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class FleetState(str, Enum):
    INIT = "initialising"
    READY = "ready"
    MISSION_ACTIVE = "mission_active"
    HANDOFF_IN_PROGRESS = "handoff_in_progress"
    EMERGENCY = "emergency"
    ALL_STOPPED = "all_stopped"
    RETURNING = "returning"


class RobotRole(str, Enum):
    INSPECTOR = "inspector"
    OPERATOR = "operator"


class RobotStatus(str, Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    INSPECTING = "inspecting"
    SETTLING = "settling"
    TARGET_FOUND = "target_found"
    WAITING_HANDOFF = "waiting_handoff"
    APPROACHING = "approaching"
    POSITIONING = "positioning"
    GRASPING = "grasping"
    PLACING = "placing"
    RETREATING = "retreating"
    RETURNING = "returning"
    STOPPED = "stopped"
    EMERGENCY = "emergency"
    ERROR = "error"


class HandoffPhase(str, Enum):
    IDLE = "idle"
    REQUESTED = "requested"
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    APPROACHING = "approaching"
    POSITIONING = "positioning"
    GRASPING = "grasping"
    PLACING = "placing"
    RETREATING = "retreating"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class RobotRecord:
    robot_id: str
    role: RobotRole
    namespace: str = ""

    status: RobotStatus = RobotStatus.IDLE
    localization_ok: bool = False
    arm_ready: bool = False
    arm_busy: bool = False
    estop: bool = False
    battery_low: bool = False
    battery_voltage: float = 0.0
    mission_id: str = ""
    waypoint_index: int = -1
    waypoint_total: int = 0
    current_workspace: str = ""
    target_workspace: str = ""

    last_heartbeat: float = 0.0
    registered_at: float = 0.0
    heartbeat_timeout_sec: float = 3.0

    @property
    def online(self) -> bool:
        return self.last_heartbeat > 0 and (
            time.monotonic() - self.last_heartbeat <= self.heartbeat_timeout_sec
        )

    @property
    def can_accept_handoff(self) -> bool:
        return (
            self.online
            and self.status == RobotStatus.IDLE
            and self.localization_ok
            and not self.estop
            and not self.battery_low
        )

    def update_heartbeat(self, data: Dict[str, Any]) -> None:
        self.last_heartbeat = time.monotonic()
        try:
            self.status = RobotStatus(str(data.get("state", self.status.value)))
        except ValueError:
            pass
        self.localization_ok = bool(data.get("localization_ok", False))
        self.arm_ready = bool(data.get("arm_ready", False))
        self.arm_busy = bool(data.get("arm_busy", False))
        self.estop = bool(data.get("estop", False))
        self.battery_low = bool(data.get("battery_low", False))
        self.battery_voltage = float(data.get("battery_voltage", 0.0))
        self.mission_id = str(data.get("mission_id", self.mission_id))
        self.waypoint_index = int(data.get("waypoint_index", self.waypoint_index))
        self.waypoint_total = int(data.get("waypoint_total", self.waypoint_total))
        self.current_workspace = str(data.get("current_workspace", self.current_workspace))
        self.target_workspace = str(data.get("target_workspace", self.target_workspace))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "role": self.role.value,
            "namespace": self.namespace,
            "status": self.status.value,
            "online": self.online,
            "localization_ok": self.localization_ok,
            "arm_ready": self.arm_ready,
            "arm_busy": self.arm_busy,
            "estop": self.estop,
            "battery_low": self.battery_low,
            "battery_voltage": self.battery_voltage,
            "mission_id": self.mission_id,
            "waypoint_index": self.waypoint_index,
            "waypoint_total": self.waypoint_total,
            "current_workspace": self.current_workspace,
            "target_workspace": self.target_workspace,
            "last_heartbeat_age_sec": (
                0.0 if not self.last_heartbeat
                else max(0.0, time.monotonic() - self.last_heartbeat)
            ),
        }


@dataclass
class HandoffRecord:
    handoff_id: str
    target_id: str
    from_robot: str
    to_robot: str
    workspace_id: str
    phase: HandoffPhase = HandoffPhase.IDLE
    requested_at: float = 0.0
    accepted_at: float = 0.0
    completed_at: float = 0.0
    success: bool = False
    error_message: str = ""
    target_data: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "handoff_id": self.handoff_id,
            "target_id": self.target_id,
            "from_robot": self.from_robot,
            "to_robot": self.to_robot,
            "workspace_id": self.workspace_id,
            "phase": self.phase.value,
            "requested_at": self.requested_at,
            "accepted_at": self.accepted_at,
            "completed_at": self.completed_at,
            "elapsed_sec": (
                0.0 if not self.requested_at
                else max(0.0, time.monotonic() - self.requested_at)
            ),
            "success": self.success,
            "error_message": self.error_message,
        }


@dataclass
class FleetModel:
    state: FleetState = FleetState.INIT
    robots: Dict[str, RobotRecord] = field(default_factory=dict)
    handoffs: Dict[str, HandoffRecord] = field(default_factory=dict)
    pending_targets: List[Dict[str, Any]] = field(default_factory=list)
    workspace_reservations: Dict[str, str] = field(default_factory=dict)
    stats: Dict[str, int] = field(default_factory=lambda: {
        "total_targets_detected": 0,
        "total_targets_handled": 0,
        "total_handoffs_completed": 0,
    })
    started_at: float = 0.0

    @property
    def inspector(self) -> Optional[RobotRecord]:
        for robot in self.robots.values():
            if robot.role == RobotRole.INSPECTOR and robot.online:
                return robot
        return None

    @property
    def operator(self) -> Optional[RobotRecord]:
        for robot in self.robots.values():
            if robot.role == RobotRole.OPERATOR and robot.online:
                return robot
        return None

    @property
    def all_robots_online(self) -> bool:
        return bool(self.robots) and all(r.online for r in self.robots.values())

    @property
    def any_estop(self) -> bool:
        return any(r.estop for r in self.robots.values())

    @property
    def active_handoff(self) -> Optional[HandoffRecord]:
        for h in self.handoffs.values():
            if h.phase not in (
                HandoffPhase.COMPLETED,
                HandoffPhase.FAILED,
                HandoffPhase.CANCELLED,
                HandoffPhase.IDLE,
            ):
                return h
        return None

    def register_robot(self, robot_id: str, role: RobotRole, namespace: str = "") -> RobotRecord:
        record = RobotRecord(
            robot_id=robot_id,
            role=role,
            namespace=namespace,
            registered_at=time.monotonic(),
        )
        self.robots[robot_id] = record
        return record

    def can_start_handoff(
        self, inspector_id: str, operator_id: str, workspace_id: str
    ):
        if self.state == FleetState.EMERGENCY:
            return False, "fleet_in_emergency"
        if self.active_handoff is not None:
            return False, "handoff_already_in_progress"
        inspector = self.robots.get(inspector_id)
        operator = self.robots.get(operator_id)
        if inspector is None or not inspector.online:
            return False, "inspector_offline"
        if operator is None or not operator.online:
            return False, "operator_offline"
        if operator.status != RobotStatus.IDLE:
            return False, "operator_not_idle"
        if operator.estop:
            return False, "operator_estop"
        if not operator.localization_ok:
            return False, "operator_no_localization"
        if workspace_id in self.workspace_reservations:
            if self.workspace_reservations[workspace_id] != operator_id:
                return False, "workspace_occupied"
        return True, ""

    def start_handoff(
        self,
        handoff_id: str,
        target_id: str,
        inspector_id: str,
        operator_id: str,
        workspace_id: str,
        target_data: Dict[str, Any],
    ) -> HandoffRecord:
        ok, reason = self.can_start_handoff(inspector_id, operator_id, workspace_id)
        if not ok:
            raise ValueError(reason)
        record = HandoffRecord(
            handoff_id=handoff_id,
            target_id=target_id,
            from_robot=inspector_id,
            to_robot=operator_id,
            workspace_id=workspace_id,
            phase=HandoffPhase.REQUESTED,
            requested_at=time.monotonic(),
            target_data=target_data,
        )
        self.handoffs[handoff_id] = record
        self.workspace_reservations[workspace_id] = operator_id
        self.state = FleetState.HANDOFF_IN_PROGRESS
        self.stats["total_targets_detected"] += 1
        return record

    def complete_handoff(self, handoff_id: str, success: bool, error: str = "") -> None:
        record = self.handoffs.get(handoff_id)
        if record is None:
            return
        record.phase = HandoffPhase.COMPLETED if success else HandoffPhase.FAILED
        record.success = success
        record.error_message = error
        record.completed_at = time.monotonic()
        self.workspace_reservations.pop(record.workspace_id, None)
        if success:
            self.stats["total_handoffs_completed"] += 1
            self.stats["total_targets_handled"] += 1
        self.state = FleetState.MISSION_ACTIVE

    def emergency_stop(self) -> None:
        self.state = FleetState.EMERGENCY
        for handoff in self.handoffs.values():
            if handoff.phase not in (
                HandoffPhase.COMPLETED,
                HandoffPhase.FAILED,
                HandoffPhase.CANCELLED,
            ):
                handoff.phase = HandoffPhase.CANCELLED

    def clear_emergency(self) -> None:
        if self.state == FleetState.EMERGENCY and not self.any_estop:
            self.state = FleetState.READY
