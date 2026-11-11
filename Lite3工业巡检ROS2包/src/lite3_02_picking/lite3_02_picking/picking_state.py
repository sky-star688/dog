"""Picking state model for Dog 2 (picking dog). ROS-free."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


class PickingState(str, Enum):
    IDLE = "idle"
    MAP_WAITING = "map_waiting"
    ASSIGNED = "assigned"
    APPROACHING = "approaching"
    POSITIONING = "positioning"
    GRASPING = "grasping"
    PLACING = "placing"
    RETREATING = "retreating"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"
    EMERGENCY = "emergency"


ACTIVE_STATES = frozenset({
    PickingState.APPROACHING,
    PickingState.POSITIONING,
    PickingState.GRASPING,
    PickingState.PLACING,
    PickingState.RETREATING,
})

TERMINAL_STATES = frozenset({PickingState.COMPLETED, PickingState.FAILED})


@dataclass
class TargetAssignment:
    handoff_id: str = ""
    target_id: str = ""
    class_name: str = "block"
    confidence: float = 0.0
    coarse_x: float = 0.0
    coarse_y: float = 0.0
    coarse_z: float = 0.0
    frame_id: str = "map"
    workspace_id: str = ""

    approach_x: float = 0.0
    approach_y: float = 0.0
    approach_yaw: float = 0.0
    centre_x: float = 0.0
    centre_y: float = 0.0
    retreat_x: float = 0.0
    retreat_y: float = 0.0
    retreat_yaw: float = 0.0

    def validate(self) -> list:
        import math

        errors = []
        if not self.handoff_id:
            errors.append("handoff_id must not be empty")
        for name in ("coarse_x", "coarse_y", "approach_x", "approach_y"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                errors.append("%s must be finite" % name)
        return errors

    def to_dict(self) -> Dict[str, Any]:
        return {
            "handoff_id": self.handoff_id,
            "target_id": self.target_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "coarse_pose": {
                "x": self.coarse_x,
                "y": self.coarse_y,
                "z": self.coarse_z,
                "frame_id": self.frame_id,
            },
            "workspace_id": self.workspace_id,
            "approach_point": {
                "x": self.approach_x,
                "y": self.approach_y,
                "yaw": self.approach_yaw,
            },
            "retreat_point": {
                "x": self.retreat_x,
                "y": self.retreat_y,
                "yaw": self.retreat_yaw,
            },
        }


@dataclass
class PickingModel:
    robot_id: str = "lite3_02"
    state: PickingState = PickingState.IDLE
    assignment: Optional[TargetAssignment] = None

    map_ready: bool = False          # frozen + hash-verified from Dog 1
    map_topic: str = "/shared_map"

    nav_ready: bool = False
    localization_ok: bool = False    # confirmed by AMCL (dog2_localization)

    openmv_ok: bool = False
    fine_position_locked: bool = False
    fine_x_mm: float = 0.0
    fine_y_mm: float = 0.0
    fine_z_mm: float = 0.0

    # Large-model (VLM) recognition result, advisory only.
    vlm_verified: bool = False
    vlm_class: str = ""
    vlm_confidence: float = 0.0
    vlm_count: int = 0
    vlm_status: str = "starting"

    arm_ready: bool = False
    arm_busy: bool = False
    pick_complete: bool = False
    place_complete: bool = False

    fleet_connected: bool = False
    estop: bool = False

    assigned_at: float = 0.0
    started_at: float = 0.0
    completed_at: float = 0.0
    error_message: str = ""

    @property
    def is_active(self) -> bool:
        return self.state in ACTIVE_STATES

    @property
    def is_terminal(self) -> bool:
        return self.state in TERMINAL_STATES

    @property
    def elapsed_sec(self) -> float:
        if self.started_at <= 0:
            return 0.0
        return max(0.0, time.monotonic() - self.started_at)

    def assign(self, assignment: TargetAssignment) -> None:
        self.assignment = assignment
        self.state = PickingState.ASSIGNED
        self.assigned_at = time.monotonic()
        self.error_message = ""

    def reset(self) -> None:
        self.state = PickingState.IDLE
        self.assignment = None
        self.pick_complete = False
        self.place_complete = False
        self.fine_position_locked = False
        self.vlm_verified = False
        self.vlm_class = ""
        self.vlm_confidence = 0.0
        self.vlm_count = 0
        self.error_message = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "state": self.state.value,
            "map_ready": self.map_ready,
            "localization_ok": self.localization_ok,
            "arm_ready": self.arm_ready,
            "arm_busy": self.arm_busy,
            "openmv_ok": self.openmv_ok,
            "vlm_verified": self.vlm_verified,
            "vlm_class": self.vlm_class,
            "vlm_confidence": self.vlm_confidence,
            "vlm_status": self.vlm_status,
            "elapsed_sec": self.elapsed_sec,
            "assignment": (
                None if self.assignment is None else self.assignment.to_dict()
            ),
            "error_message": self.error_message,
        }
