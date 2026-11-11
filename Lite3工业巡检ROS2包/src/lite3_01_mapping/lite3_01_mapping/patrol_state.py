"""Patrol domain model for Dog 1 (mapping + patrol dog). ROS-free."""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class PatrolState(str, Enum):
    IDLE = "idle"
    MAPPING = "mapping"
    PATROLLING = "patrolling"
    SETTLING = "settling"
    INSPECTING = "inspecting"
    TARGET_FOUND = "target_found"
    WAITING_HANDOFF = "waiting_handoff"
    RESUMING = "resuming"
    RETURNING = "returning"
    STOPPED = "stopped"
    EMERGENCY = "emergency"


ACTIVE_STATES = frozenset({
    PatrolState.MAPPING,
    PatrolState.PATROLLING,
    PatrolState.SETTLING,
    PatrolState.INSPECTING,
    PatrolState.RETURNING,
})


@dataclass
class PatrolWaypoint:
    name: str
    frame_id: str
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    inspect: bool = True
    settle_sec: float = 2.0
    tolerance_xy: float = 0.30
    timeout_sec: float = 120.0

    def validate(self) -> List[str]:
        errors = []
        if not self.name:
            errors.append("name must not be empty")
        if not self.frame_id:
            errors.append("frame_id must not be empty")
        for name in ("x", "y", "z", "yaw"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                errors.append("%s must be finite" % name)
        return errors


@dataclass
class PatrolTarget:
    target_id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    class_name: str = "unknown"
    confidence: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    stable_frames: int = 0
    waypoint_name: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "target_id": self.target_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "stable_frames": self.stable_frames,
            "waypoint_name": self.waypoint_name,
            "requires_operation": self.class_name == "block",
        }


@dataclass
class PatrolModel:
    robot_id: str = "lite3_01"
    state: PatrolState = PatrolState.IDLE
    waypoints: List[PatrolWaypoint] = field(default_factory=list)
    current_index: int = 0

    map_ready: bool = False
    detector_ok: bool = False
    depth_ok: bool = False
    localization_ok: bool = False

    current_target: Optional[PatrolTarget] = None
    pending_target: bool = False

    fleet_connected: bool = False
    handoff_pending: bool = False
    handoff_complete: bool = False
    handoff_success: bool = False

    estop: bool = False

    started_at: float = 0.0
    waypoint_started_at: float = 0.0
    settle_until: float = 0.0

    waypoints_visited: int = 0
    targets_detected: int = 0
    targets_reported: int = 0

    @property
    def current_waypoint(self) -> Optional[PatrolWaypoint]:
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    @property
    def is_active(self) -> bool:
        return self.state in ACTIVE_STATES

    @property
    def patrol_complete(self) -> bool:
        return bool(self.waypoints) and self.current_index >= len(self.waypoints)

    def set_waypoints(self, waypoints: List[PatrolWaypoint]) -> None:
        errors = []
        for waypoint in waypoints:
            errors.extend(waypoint.validate())
        if errors:
            raise ValueError("; ".join(errors))
        self.waypoints = list(waypoints)
        self.current_index = 0

    def set_target(
        self, class_name: str, confidence: float, x: float, y: float, z: float = 0.0
    ) -> None:
        self.current_target = PatrolTarget(
            class_name=class_name,
            confidence=confidence,
            x=x,
            y=y,
            z=z,
            stable_frames=5,
            waypoint_name=self.current_waypoint.name if self.current_waypoint else "",
        )
        self.pending_target = True
        self.state = PatrolState.TARGET_FOUND

    def clear_target(self) -> None:
        self.current_target = None
        self.pending_target = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "state": self.state.value,
            "map_ready": self.map_ready,
            "localization_ok": self.localization_ok,
            "detector_ok": self.detector_ok,
            "depth_ok": self.depth_ok,
            "waypoint_index": self.current_index,
            "waypoint_total": len(self.waypoints),
            "current_waypoint": (
                self.current_waypoint.name if self.current_waypoint else None
            ),
            "pending_target": self.pending_target,
            "fleet_connected": self.fleet_connected,
            "targets_detected": self.targets_detected,
            "targets_reported": self.targets_reported,
            "waypoints_visited": self.waypoints_visited,
        }
