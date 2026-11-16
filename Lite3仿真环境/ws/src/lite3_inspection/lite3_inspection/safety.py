"""Dependency-free safety limits and command arbitration.

ROS nodes are thin adapters: decision logic lives here so it is testable
without ROS or hardware.  Rules:
  - non-finite input is rejected as a whole (never partially forwarded)
  - every axis is clamped to its limit
  - stale commands are never forwarded; no fresh command means zero
  - estop always wins, disabled always means zero
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class TwistValues:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float, float, float, float]:
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )

    def is_finite(self) -> bool:
        return all(math.isfinite(value) for value in self.as_tuple())

    def is_zero(self, epsilon: float = 1e-9) -> bool:
        return all(abs(value) <= epsilon for value in self.as_tuple())


ZERO_TWIST = TwistValues()


@dataclass(frozen=True)
class SafetyLimits:
    max_linear_x: float = 0.30
    max_linear_y: float = 0.15
    max_linear_z: float = 0.0
    max_angular_x: float = 0.0
    max_angular_y: float = 0.0
    max_angular_z: float = 0.50
    timeout_sec: float = 0.25
    publish_hz: float = 20.0

    def validate(self) -> List[str]:
        errors = []
        for name, value in (
            ("max_linear_x", self.max_linear_x),
            ("max_linear_y", self.max_linear_y),
            ("max_linear_z", self.max_linear_z),
            ("max_angular_x", self.max_angular_x),
            ("max_angular_y", self.max_angular_y),
            ("max_angular_z", self.max_angular_z),
        ):
            if not math.isfinite(value) or value < 0.0:
                errors.append("%s must be finite and non-negative" % name)
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            errors.append("timeout_sec must be positive")
        if not math.isfinite(self.publish_hz) or self.publish_hz <= 0.0:
            errors.append("publish_hz must be positive")
        return errors


@dataclass(frozen=True)
class ClampResult:
    values: TwistValues
    valid_input: bool
    clamped: bool
    violations: Tuple[str, ...]


def clamp_twist(values: TwistValues, limits: SafetyLimits) -> ClampResult:
    errors = limits.validate()
    if errors:
        raise ValueError("; ".join(errors))
    if not values.is_finite():
        return ClampResult(ZERO_TWIST, False, False, ("non_finite",))

    violations = []

    def apply(name: str, value: float, limit: float) -> float:
        if limit == 0.0:
            if value != 0.0:
                violations.append(name)
            return 0.0
        if value > limit:
            violations.append(name + "_high")
            return limit
        if value < -limit:
            violations.append(name + "_low")
            return -limit
        return value

    safe = TwistValues(
        linear_x=apply("linear_x", values.linear_x, limits.max_linear_x),
        linear_y=apply("linear_y", values.linear_y, limits.max_linear_y),
        linear_z=apply("linear_z", values.linear_z, limits.max_linear_z),
        angular_x=apply("angular_x", values.angular_x, limits.max_angular_x),
        angular_y=apply("angular_y", values.angular_y, limits.max_angular_y),
        angular_z=apply("angular_z", values.angular_z, limits.max_angular_z),
    )
    return ClampResult(safe, True, bool(violations), tuple(violations))


@dataclass(frozen=True)
class CommandSample:
    source: str
    priority: int
    stamp_sec: float
    values: TwistValues
    valid: bool = True


@dataclass(frozen=True)
class Selection:
    source: str
    values: TwistValues
    reason: str
    active: bool


class CommandArbiter:
    """Select the highest-priority fresh, valid command."""

    def __init__(
        self,
        timeouts: Optional[Dict[str, float]] = None,
        default_timeout_sec: float = 0.30,
    ) -> None:
        self._timeouts = dict(timeouts or {})
        if default_timeout_sec <= 0.0 or not math.isfinite(default_timeout_sec):
            raise ValueError("default_timeout_sec must be positive")
        self._default_timeout = float(default_timeout_sec)

    def timeout_for(self, source: str) -> float:
        return float(self._timeouts.get(source, self._default_timeout))

    def select(
        self,
        now_sec: float,
        samples: Sequence[CommandSample],
        enabled: bool = True,
        estop: bool = False,
    ) -> Selection:
        if estop:
            return Selection("estop", ZERO_TWIST, "emergency_stop", False)
        if not enabled:
            return Selection("disabled", ZERO_TWIST, "motion_disabled", False)

        candidates = []
        for sample in samples:
            if not sample.valid or not sample.values.is_finite():
                continue
            age = float(now_sec) - float(sample.stamp_sec)
            if age < 0.0:
                continue
            if age > self.timeout_for(sample.source):
                continue
            candidates.append(sample)

        if not candidates:
            return Selection("timeout", ZERO_TWIST, "no_fresh_command", False)
        chosen = max(candidates, key=lambda item: (item.priority, item.stamp_sec))
        return Selection(chosen.source, chosen.values, "selected", True)


def twist_from_sequence(values: Sequence[float]) -> TwistValues:
    if len(values) != 6:
        raise ValueError("expected six Twist components")
    return TwistValues(*(float(value) for value in values))
