"""Waypoint mission domain model and deterministic state machine.

No ROS dependency: a ROS node supplies a NavigationGateway implementation and
forwards Nav2 action callbacks into the runner.  Call all methods from one
executor thread.
"""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol, Sequence


class MissionState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SETTLING = "settling"
    PAUSED = "paused"
    CANCELLED = "cancelled"
    COMPLETED = "completed"
    FAILED = "failed"


ACTIVE_STATES = (MissionState.RUNNING, MissionState.SETTLING)
TERMINAL_STATES = (MissionState.CANCELLED, MissionState.COMPLETED, MissionState.FAILED)


class NavigationGateway(Protocol):
    def send_goal(self, waypoint: "Waypoint", goal_id: str) -> None:
        ...

    def cancel(self) -> None:
        ...


@dataclass(frozen=True)
class Waypoint:
    name: str
    frame_id: str
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    tolerance_xy: float = 0.30
    tolerance_yaw: float = 0.35
    settle_sec: float = 2.0
    timeout_sec: float = 120.0
    max_retries: int = 1
    inspect: bool = True

    def validate(self) -> List[str]:
        errors = []
        if not self.name or not self.name.strip():
            errors.append("waypoint.name must not be empty")
        if not self.frame_id or not self.frame_id.strip():
            errors.append("%s.frame_id must not be empty" % self.name)
        for field_name in ("x", "y", "z", "yaw"):
            value = getattr(self, field_name)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                errors.append("%s.%s must be finite" % (self.name, field_name))
        if self.tolerance_xy < 0.0 or not math.isfinite(self.tolerance_xy):
            errors.append("%s.tolerance_xy must be non-negative" % self.name)
        if self.tolerance_yaw < 0.0 or not math.isfinite(self.tolerance_yaw):
            errors.append("%s.tolerance_yaw must be non-negative" % self.name)
        if self.settle_sec < 0.0 or not math.isfinite(self.settle_sec):
            errors.append("%s.settle_sec must be non-negative" % self.name)
        if self.timeout_sec <= 0.0 or not math.isfinite(self.timeout_sec):
            errors.append("%s.timeout_sec must be positive" % self.name)
        if not isinstance(self.max_retries, int) or self.max_retries < 0:
            errors.append("%s.max_retries must be a non-negative integer" % self.name)
        return errors


@dataclass(frozen=True)
class MissionConfig:
    mission_id: str
    robot_id: str
    map_id: str
    map_version: str
    frame_id: str
    waypoints: Sequence[Waypoint]
    stop_on_failure: bool = True

    def validate(self) -> List[str]:
        errors = []
        for field_name in ("mission_id", "robot_id", "map_id", "map_version", "frame_id"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value.strip():
                errors.append("%s must be a non-empty string" % field_name)
        if not self.waypoints:
            errors.append("at least one waypoint is required")
        names = [waypoint.name for waypoint in self.waypoints]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            errors.append("duplicate waypoint names: %s" % ", ".join(duplicates))
        for waypoint in self.waypoints:
            errors.extend(waypoint.validate())
        return errors

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionConfig":
        if not isinstance(data, dict):
            raise ValueError("mission config must be a mapping")
        mission = data.get("mission", data)
        if not isinstance(mission, dict):
            raise ValueError("mission must be a mapping")
        raw_waypoints = mission.get("waypoints", [])
        if not isinstance(raw_waypoints, list):
            raise ValueError("mission.waypoints must be a list")

        defaults = {
            "tolerance_xy": float(mission.get("default_tolerance_xy", 0.30)),
            "tolerance_yaw": float(mission.get("default_tolerance_yaw", 0.35)),
            "settle_sec": float(mission.get("default_settle_sec", 2.0)),
            "timeout_sec": float(mission.get("default_timeout_sec", 120.0)),
            "max_retries": int(mission.get("default_max_retries", 1)),
        }
        waypoints = []
        for index, raw in enumerate(raw_waypoints):
            if not isinstance(raw, dict):
                raise ValueError("waypoint %d must be a mapping" % index)
            values = dict(defaults)
            values.update({key: raw[key] for key in raw if key in values})
            missing = [key for key in ("name", "frame_id", "x", "y") if key not in raw]
            if missing:
                raise ValueError(
                    "waypoint %d is missing required fields: %s"
                    % (index, ", ".join(missing))
                )
            waypoints.append(
                Waypoint(
                    name=str(raw["name"]),
                    frame_id=str(raw["frame_id"]),
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    z=float(raw.get("z", 0.0)),
                    yaw=float(raw.get("yaw", 0.0)),
                    tolerance_xy=float(values["tolerance_xy"]),
                    tolerance_yaw=float(values["tolerance_yaw"]),
                    settle_sec=float(values["settle_sec"]),
                    timeout_sec=float(values["timeout_sec"]),
                    max_retries=int(values["max_retries"]),
                    inspect=bool(raw.get("inspect", True)),
                )
            )

        config = cls(
            mission_id=str(mission.get("mission_id", "")),
            robot_id=str(mission.get("robot_id", "")),
            map_id=str(mission.get("map_id", "")),
            map_version=str(mission.get("map_version", "")),
            frame_id=str(mission.get("frame_id", "map")),
            waypoints=waypoints,
            stop_on_failure=bool(mission.get("stop_on_failure", True)),
        )
        errors = config.validate()
        if errors:
            raise ValueError("; ".join(errors))
        return config


@dataclass
class WaypointResult:
    index: int
    name: str
    success: bool
    attempts: int
    started_at: Optional[float]
    finished_at: Optional[float]
    error: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index,
            "name": self.name,
            "success": self.success,
            "attempts": self.attempts,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "error": self.error,
            "duration_sec": (
                None
                if self.started_at is None or self.finished_at is None
                else max(0.0, self.finished_at - self.started_at)
            ),
        }


class MissionRunner:
    """Execute ordered waypoints with retry, timeout, pause and cancel."""

    def __init__(
        self,
        config: MissionConfig,
        navigation: NavigationGateway,
        clock=time.monotonic,
    ) -> None:
        errors = config.validate()
        if errors:
            raise ValueError("; ".join(errors))
        self.config = config
        self.navigation = navigation
        self.clock = clock
        self._state = MissionState.IDLE
        self._index = 0
        self._attempts: List[int] = [0 for _ in config.waypoints]
        self._results: List[WaypointResult] = []
        self._active_goal_id: Optional[str] = None
        self._started_at: Optional[float] = None
        self._waypoint_started_at: Optional[float] = None
        self._deadline: Optional[float] = None
        self._settle_until: Optional[float] = None
        self._last_error = ""
        self._paused_from: Optional[MissionState] = None
        self._paused_settle_remaining = 0.0

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if self._index >= len(self.config.waypoints):
            return None
        return self.config.waypoints[self._index]

    @property
    def results(self) -> Sequence[WaypointResult]:
        return tuple(self._results)

    def start(self, now: Optional[float] = None) -> None:
        if self._state not in (MissionState.IDLE,) + TERMINAL_STATES:
            raise RuntimeError("cannot start mission from state %s" % self._state.value)
        now = self.clock() if now is None else float(now)
        self._state = MissionState.RUNNING
        self._index = 0
        self._attempts = [0 for _ in self.config.waypoints]
        self._results = []
        self._active_goal_id = None
        self._started_at = now
        self._waypoint_started_at = None
        self._deadline = None
        self._settle_until = None
        self._last_error = ""
        self._paused_from = None
        self._paused_settle_remaining = 0.0
        self._start_current(now)

    def _start_current(self, now: float) -> None:
        if self._index >= len(self.config.waypoints):
            self._state = MissionState.COMPLETED
            self._active_goal_id = None
            return
        waypoint = self.config.waypoints[self._index]
        self._attempts[self._index] += 1
        goal_id = "%d:%d:%s" % (self._index, self._attempts[self._index], uuid.uuid4().hex)
        self._active_goal_id = goal_id
        self._waypoint_started_at = now
        self._deadline = now + waypoint.timeout_sec
        self._settle_until = None
        self._state = MissionState.RUNNING
        try:
            self.navigation.send_goal(waypoint, goal_id)
        except Exception as exc:
            self._active_goal_id = None
            self._handle_failure(now, "navigation_send_failed: %s" % exc)

    def on_navigation_succeeded(self, now: Optional[float], goal_id: str) -> None:
        if goal_id != self._active_goal_id or self._state != MissionState.RUNNING:
            return
        now = self.clock() if now is None else float(now)
        self._active_goal_id = None
        self._deadline = None
        self._settle_until = now + self.config.waypoints[self._index].settle_sec
        if self.config.waypoints[self._index].settle_sec <= 0.0:
            self._complete_current(now)
        else:
            self._state = MissionState.SETTLING

    def on_navigation_failed(self, now: Optional[float], goal_id: str, reason: str = "navigation_failed") -> None:
        if goal_id != self._active_goal_id or self._state != MissionState.RUNNING:
            return
        now = self.clock() if now is None else float(now)
        self._active_goal_id = None
        self._deadline = None
        self._handle_failure(now, reason)

    def tick(self, now: Optional[float] = None) -> None:
        now = self.clock() if now is None else float(now)
        if self._state == MissionState.RUNNING:
            if self._deadline is not None and now >= self._deadline:
                self._active_goal_id = None
                self._deadline = None
                try:
                    self.navigation.cancel()
                except Exception:
                    pass
                self._handle_failure(now, "waypoint_timeout")
        elif self._state == MissionState.SETTLING:
            if self._settle_until is not None and now >= self._settle_until:
                self._complete_current(now)

    def _complete_current(self, now: float) -> None:
        if self._index >= len(self.config.waypoints):
            self._state = MissionState.COMPLETED
            return
        self._record_result(True, now, "")
        self._index += 1
        self._start_current(now)

    def _handle_failure(self, now: float, reason: str) -> None:
        waypoint = self.config.waypoints[self._index]
        self._last_error = reason
        if self._attempts[self._index] <= waypoint.max_retries:
            self._start_current(now)
            return
        self._record_result(False, now, reason)
        if self.config.stop_on_failure:
            self._active_goal_id = None
            self._deadline = None
            self._settle_until = None
            self._state = MissionState.FAILED
            return
        self._index += 1
        self._start_current(now)

    def _record_result(self, success: bool, now: float, error: str) -> None:
        if self._index >= len(self.config.waypoints):
            return
        waypoint = self.config.waypoints[self._index]
        self._results.append(
            WaypointResult(
                index=self._index,
                name=waypoint.name,
                success=success,
                attempts=self._attempts[self._index],
                started_at=self._waypoint_started_at,
                finished_at=now,
                error=error,
            )
        )

    def pause(self, now: Optional[float] = None) -> bool:
        if self._state not in ACTIVE_STATES:
            return False
        now = self.clock() if now is None else float(now)
        self._paused_from = self._state
        self._paused_settle_remaining = (
            max(0.0, self._settle_until - now)
            if self._state == MissionState.SETTLING and self._settle_until is not None
            else 0.0
        )
        self._active_goal_id = None
        self._deadline = None
        self._settle_until = None
        try:
            self.navigation.cancel()
        except Exception:
            pass
        self._state = MissionState.PAUSED
        return True

    def resume(self, now: Optional[float] = None) -> bool:
        if self._state != MissionState.PAUSED:
            return False
        now = self.clock() if now is None else float(now)
        paused_from = self._paused_from
        remaining = self._paused_settle_remaining
        self._paused_from = None
        self._paused_settle_remaining = 0.0
        self._settle_until = None
        if paused_from == MissionState.SETTLING:
            self._state = MissionState.SETTLING
            self._settle_until = now + remaining
            return True
        self._state = MissionState.RUNNING
        self._start_current(now)
        return True

    def cancel(self, now: Optional[float] = None) -> bool:
        if self._state not in ACTIVE_STATES + (MissionState.PAUSED,):
            return False
        now = self.clock() if now is None else float(now)
        self._active_goal_id = None
        self._deadline = None
        self._settle_until = None
        try:
            self.navigation.cancel()
        except Exception:
            pass
        if self._index < len(self.config.waypoints):
            self._record_result(False, now, "cancelled")
        self._last_error = "cancelled"
        self._state = MissionState.CANCELLED
        return True

    def status_dict(self) -> Dict[str, Any]:
        waypoint = (
            self.config.waypoints[self._index]
            if self._index < len(self.config.waypoints)
            else None
        )
        return {
            "state": self._state.value,
            "mission_id": self.config.mission_id,
            "robot_id": self.config.robot_id,
            "map_id": self.config.map_id,
            "map_version": self.config.map_version,
            "frame_id": self.config.frame_id,
            "index": self._index,
            "total": len(self.config.waypoints),
            "current_waypoint": None if waypoint is None else waypoint.name,
            "attempts": self._attempts,
            "results": [result.to_dict() for result in self._results],
            "last_error": self._last_error,
            "started_at": self._started_at,
            "paused_from": None if self._paused_from is None else self._paused_from.value,
        }
