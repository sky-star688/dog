"""Handoff state machine: serial inspector-to-operator target transfer."""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List


class HandoffError(Exception):
    def __init__(self, reason: str, error_code: int = 1) -> None:
        super().__init__(reason)
        self.reason = reason
        self.error_code = error_code


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


ACTIVE_PHASES = frozenset({
    HandoffPhase.REQUESTED,
    HandoffPhase.ACCEPTED,
    HandoffPhase.APPROACHING,
    HandoffPhase.POSITIONING,
    HandoffPhase.GRASPING,
    HandoffPhase.PLACING,
    HandoffPhase.RETREATING,
})

TERMINAL_PHASES = frozenset({
    HandoffPhase.COMPLETED,
    HandoffPhase.FAILED,
    HandoffPhase.CANCELLED,
})


@dataclass
class HandoffTarget:
    target_id: str
    class_name: str
    confidence: float
    coarse_x: float
    coarse_y: float
    coarse_z: float
    coarse_yaw: float
    frame_id: str = "map"
    inspector_id: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)

    def validate(self) -> List[str]:
        errors = []
        if not self.target_id or not isinstance(self.target_id, str):
            errors.append("target_id must be a non-empty string")
        if not self.class_name:
            errors.append("class_name must not be empty")
        if not 0.0 <= self.confidence <= 1.0:
            errors.append("confidence must be in [0, 1]")
        for name in ("coarse_x", "coarse_y", "coarse_z", "coarse_yaw"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                errors.append("%s must be finite" % name)
        return errors

    def to_dict(self) -> Dict[str, Any]:
        return {
            "target_id": self.target_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "coarse_x": self.coarse_x,
            "coarse_y": self.coarse_y,
            "coarse_z": self.coarse_z,
            "coarse_yaw": self.coarse_yaw,
            "frame_id": self.frame_id,
            "inspector_id": self.inspector_id,
            "extra": self.extra,
        }


@dataclass
class HandoffState:
    handoff_id: str
    target: HandoffTarget
    from_robot: str
    to_robot: str
    workspace_id: str
    phase: HandoffPhase = HandoffPhase.IDLE
    created_at: float = field(default_factory=time.monotonic)
    accepted_at: float = 0.0
    completed_at: float = 0.0
    error_message: str = ""
    error_code: int = 0
    fine_x: float = 0.0
    fine_y: float = 0.0
    fine_z: float = 0.0
    fine_valid: bool = False

    @classmethod
    def create(
        cls, target: HandoffTarget, from_robot: str, to_robot: str, workspace_id: str
    ) -> "HandoffState":
        return cls(
            handoff_id="handoff_%s_%s" % (to_robot, uuid.uuid4().hex[:8]),
            target=target,
            from_robot=from_robot,
            to_robot=to_robot,
            workspace_id=workspace_id,
            phase=HandoffPhase.REQUESTED,
        )

    @property
    def is_active(self) -> bool:
        return self.phase in ACTIVE_PHASES

    @property
    def is_terminal(self) -> bool:
        return self.phase in TERMINAL_PHASES

    @property
    def elapsed_sec(self) -> float:
        return max(0.0, time.monotonic() - self.created_at)

    def accept(self) -> None:
        if self.phase != HandoffPhase.REQUESTED:
            raise HandoffError(
                "handoff must be REQUESTED, current: %s" % self.phase.value, 1
            )
        self.phase = HandoffPhase.ACCEPTED
        self.accepted_at = time.monotonic()

    def reject(self, reason: str) -> None:
        if self.phase != HandoffPhase.REQUESTED:
            raise HandoffError(
                "handoff must be REQUESTED, current: %s" % self.phase.value, 1
            )
        self.phase = HandoffPhase.REJECTED
        self.error_message = reason
        self.completed_at = time.monotonic()

    def transition_to(self, phase: HandoffPhase) -> None:
        allowed = {
            HandoffPhase.REQUESTED: {HandoffPhase.ACCEPTED, HandoffPhase.REJECTED, HandoffPhase.CANCELLED},
            HandoffPhase.ACCEPTED: {HandoffPhase.APPROACHING},
            HandoffPhase.APPROACHING: {HandoffPhase.POSITIONING, HandoffPhase.FAILED, HandoffPhase.CANCELLED},
            HandoffPhase.POSITIONING: {HandoffPhase.GRASPING, HandoffPhase.FAILED, HandoffPhase.CANCELLED},
            HandoffPhase.GRASPING: {HandoffPhase.PLACING, HandoffPhase.FAILED, HandoffPhase.CANCELLED},
            HandoffPhase.PLACING: {HandoffPhase.RETREATING, HandoffPhase.COMPLETED, HandoffPhase.FAILED, HandoffPhase.CANCELLED},
            HandoffPhase.RETREATING: {HandoffPhase.COMPLETED, HandoffPhase.FAILED},
        }
        if phase not in allowed.get(self.phase, set()):
            raise HandoffError(
                "cannot transition %s -> %s" % (self.phase.value, phase.value), 1
            )
        self.phase = phase
        if self.is_terminal:
            self.completed_at = time.monotonic()

    def set_fine_position(self, x: float, y: float, z: float) -> None:
        self.fine_x = float(x)
        self.fine_y = float(y)
        self.fine_z = float(z)
        self.fine_valid = True

    def fail(self, reason: str, error_code: int = 1) -> None:
        self.phase = HandoffPhase.FAILED
        self.error_message = reason
        self.error_code = error_code
        self.completed_at = time.monotonic()

    def cancel(self) -> None:
        self.phase = HandoffPhase.CANCELLED
        self.completed_at = time.monotonic()

    def complete(self) -> None:
        self.phase = HandoffPhase.COMPLETED
        self.completed_at = time.monotonic()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "handoff_id": self.handoff_id,
            "target_id": self.target.target_id,
            "target_class": self.target.class_name,
            "target_confidence": self.target.confidence,
            "from_robot": self.from_robot,
            "to_robot": self.to_robot,
            "workspace_id": self.workspace_id,
            "phase": self.phase.value,
            "created_at": self.created_at,
            "accepted_at": self.accepted_at,
            "completed_at": self.completed_at,
            "elapsed_sec": self.elapsed_sec,
            "error_message": self.error_message,
            "error_code": self.error_code,
            "fine_x": self.fine_x,
            "fine_y": self.fine_y,
            "fine_z": self.fine_z,
            "fine_valid": self.fine_valid,
            "coarse_pose": {
                "x": self.target.coarse_x,
                "y": self.target.coarse_y,
                "z": self.target.coarse_z,
                "yaw": self.target.coarse_yaw,
                "frame_id": self.target.frame_id,
            },
        }
