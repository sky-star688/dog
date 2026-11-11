"""Workspace reservation for dual-dog cooperative zones (ROS-free)."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class WorkspaceState(str, Enum):
    FREE = "free"
    RESERVED = "reserved"
    OCCUPIED = "occupied"
    RELEASING = "releasing"


@dataclass
class WorkspaceGeometry:
    workspace_id: str
    approach_x: float
    approach_y: float
    approach_yaw: float
    centre_x: float
    centre_y: float
    retreat_x: float
    retreat_y: float
    retreat_yaw: float
    boundary: List[List[float]] = field(default_factory=list)
    frame_id: str = "map"
    max_occupancy_sec: float = 300.0
    reservation_timeout_sec: float = 600.0

    def validate(self) -> List[str]:
        errors = []
        if not self.workspace_id or not isinstance(self.workspace_id, str):
            errors.append("workspace_id must be a non-empty string")
        for name in (
            "approach_x", "approach_y", "approach_yaw",
            "centre_x", "centre_y",
            "retreat_x", "retreat_y", "retreat_yaw",
        ):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                errors.append("%s must be finite" % name)
        if self.max_occupancy_sec <= 0:
            errors.append("max_occupancy_sec must be positive")
        if self.reservation_timeout_sec <= 0:
            errors.append("reservation_timeout_sec must be positive")
        return errors

    def to_dict(self) -> Dict[str, Any]:
        return {
            "workspace_id": self.workspace_id,
            "approach": {"x": self.approach_x, "y": self.approach_y, "yaw": self.approach_yaw},
            "centre": {"x": self.centre_x, "y": self.centre_y},
            "retreat": {"x": self.retreat_x, "y": self.retreat_y, "yaw": self.retreat_yaw},
            "boundary": self.boundary,
            "frame_id": self.frame_id,
            "max_occupancy_sec": self.max_occupancy_sec,
            "reservation_timeout_sec": self.reservation_timeout_sec,
        }


@dataclass
class WorkspaceReservation:
    workspace_id: str
    reserved_by: str = ""
    reserved_for_target: str = ""
    state: WorkspaceState = WorkspaceState.FREE
    reserved_at: float = 0.0
    expires_at: float = 0.0
    occupied_at: float = 0.0

    @property
    def is_active(self) -> bool:
        return self.state in (WorkspaceState.RESERVED, WorkspaceState.OCCUPIED)

    @property
    def is_expired(self) -> bool:
        if not self.is_active or self.expires_at <= 0:
            return False
        return time.monotonic() > self.expires_at

    @property
    def age_sec(self) -> float:
        if self.reserved_at <= 0:
            return 0.0
        return max(0.0, time.monotonic() - self.reserved_at)


class WorkspaceManager:
    def __init__(self, workspaces: Optional[Dict[str, WorkspaceGeometry]] = None):
        self._workspaces: Dict[str, WorkspaceGeometry] = dict(workspaces or {})
        self._reservations: Dict[str, WorkspaceReservation] = {
            ws_id: WorkspaceReservation(workspace_id=ws_id) for ws_id in self._workspaces
        }

    def register_workspace(self, geometry: WorkspaceGeometry) -> None:
        geometry.validate()
        self._workspaces[geometry.workspace_id] = geometry
        if geometry.workspace_id not in self._reservations:
            self._reservations[geometry.workspace_id] = WorkspaceReservation(
                workspace_id=geometry.workspace_id
            )

    def reserve(
        self,
        workspace_id: str,
        robot_id: str,
        target_id: str,
        duration_sec: float = 300.0,
    ) -> WorkspaceReservation:
        geometry = self._workspaces.get(workspace_id)
        if geometry is None:
            raise ValueError("unknown workspace: %s" % workspace_id)
        reservation = self._reservations.setdefault(
            workspace_id, WorkspaceReservation(workspace_id=workspace_id)
        )
        if reservation.is_active:
            if reservation.reserved_by == robot_id:
                return reservation
            raise ValueError(
                "workspace %s already reserved by %s"
                % (workspace_id, reservation.reserved_by)
            )
        now = time.monotonic()
        reservation.reserved_by = robot_id
        reservation.reserved_for_target = target_id
        reservation.state = WorkspaceState.RESERVED
        reservation.reserved_at = now
        reservation.expires_at = now + min(duration_sec, geometry.reservation_timeout_sec)
        return reservation

    def occupy(self, workspace_id: str, robot_id: str) -> WorkspaceReservation:
        reservation = self._reservations.get(workspace_id)
        if reservation is None:
            raise ValueError("unknown workspace: %s" % workspace_id)
        if reservation.reserved_by != robot_id:
            raise ValueError(
                "workspace %s reserved by %s, not %s"
                % (workspace_id, reservation.reserved_by, robot_id)
            )
        if reservation.state != WorkspaceState.RESERVED:
            raise ValueError("workspace %s is not RESERVED" % workspace_id)
        reservation.state = WorkspaceState.OCCUPIED
        reservation.occupied_at = time.monotonic()
        return reservation

    def release(self, workspace_id: str, robot_id: str) -> None:
        reservation = self._reservations.get(workspace_id)
        if reservation is None:
            return
        if reservation.reserved_by != robot_id:
            raise ValueError(
                "workspace %s reserved by %s, not %s"
                % (workspace_id, reservation.reserved_by, robot_id)
            )
        reservation.state = WorkspaceState.FREE
        reservation.reserved_by = ""
        reservation.reserved_for_target = ""

    def force_release_expired(self) -> List[str]:
        released = []
        for ws_id, reservation in self._reservations.items():
            if reservation.is_expired:
                reservation.state = WorkspaceState.FREE
                reservation.reserved_by = ""
                reservation.reserved_for_target = ""
                released.append(ws_id)
        return released

    def get_reservation(self, workspace_id: str) -> Optional[WorkspaceReservation]:
        return self._reservations.get(workspace_id)

    def get_geometry(self, workspace_id: str) -> Optional[WorkspaceGeometry]:
        return self._workspaces.get(workspace_id)

    def list_workspaces(self) -> List[str]:
        return sorted(self._workspaces.keys())

    def is_workspace_free(self, workspace_id: str) -> bool:
        reservation = self._reservations.get(workspace_id)
        return reservation is None or reservation.state == WorkspaceState.FREE

    def to_dict(self) -> Dict[str, Any]:
        result = {}
        for ws_id, workspace in self._workspaces.items():
            reservation = self._reservations.get(ws_id)
            result[ws_id] = {
                "geometry": workspace.to_dict(),
                "reservation": None
                if reservation is None
                else {
                    "state": reservation.state.value,
                    "reserved_by": reservation.reserved_by,
                    "reserved_for_target": reservation.reserved_for_target,
                    "reserved_at": reservation.reserved_at,
                    "expires_at": reservation.expires_at,
                    "age_sec": reservation.age_sec,
                    "is_expired": reservation.is_expired,
                },
            }
        return result
