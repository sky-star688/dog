"""Cross-dog safety interlocks for the dual-Lite3 fleet (ROS-free)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class SafetyDistance:
    robot_to_robot_min_m: float = 3.0
    inspector_to_workspace_min_m: float = 5.0
    operator_approach_offset_m: float = 2.0


@dataclass(frozen=True)
class FleetSafetyLimits:
    max_handoff_duration_sec: float = 600.0
    max_operator_response_sec: float = 30.0
    battery_warning_voltage: float = 22.0
    battery_critical_voltage: float = 21.0
    localization_loss_timeout_sec: float = 2.0
    heartbeat_loss_timeout_sec: float = 3.0


class FleetSafetyChecker:
    def __init__(
        self,
        distances: SafetyDistance = None,
        limits: FleetSafetyLimits = None,
    ):
        self.distances = distances or SafetyDistance()
        self.limits = limits or FleetSafetyLimits()

    def check_workspace_intrusion(
        self,
        robot_x: float,
        robot_y: float,
        workspace_centre_x: float,
        workspace_centre_y: float,
        robot_role: str,
        workspace_state: str,
    ) -> Tuple[bool, str]:
        dist = math.hypot(robot_x - workspace_centre_x, robot_y - workspace_centre_y)
        if robot_role == "inspector" and workspace_state in ("reserved", "occupied"):
            if dist < self.distances.inspector_to_workspace_min_m:
                return False, (
                    "inspector too close to %s workspace (%.2f m < %.2f m)"
                    % (
                        workspace_state,
                        dist,
                        self.distances.inspector_to_workspace_min_m,
                    )
                )
        return True, ""

    def check_robot_separation(
        self,
        x1: float, y1: float, id1: str,
        x2: float, y2: float, id2: str,
    ) -> Tuple[bool, str]:
        dist = math.hypot(x1 - x2, y1 - y2)
        if dist < self.distances.robot_to_robot_min_m:
            return False, (
                "robots %s and %s too close (%.2f m < %.2f m)"
                % (id1, id2, dist, self.distances.robot_to_robot_min_m)
            )
        return True, ""

    def check_battery(self, voltage: float, robot_id: str) -> Tuple[bool, str, str]:
        if voltage <= self.limits.battery_critical_voltage:
            return False, (
                "%s battery critical (%.1f V)" % (robot_id, voltage)
            ), "emergency_stop"
        if voltage <= self.limits.battery_warning_voltage:
            return False, (
                "%s battery low (%.1f V)" % (robot_id, voltage)
            ), "return_to_base"
        return True, "", ""

    def check_handoff_timeout(self, elapsed_sec: float) -> Tuple[bool, str]:
        if elapsed_sec > self.limits.max_handoff_duration_sec:
            return False, (
                "handoff exceeded maximum duration (%.1f s > %.1f s)"
                % (elapsed_sec, self.limits.max_handoff_duration_sec)
            )
        return True, ""

    def check_operator_response(self, elapsed_sec: float) -> Tuple[bool, str]:
        if elapsed_sec > self.limits.max_operator_response_sec:
            return False, (
                "operator response timeout (%.1f s > %.1f s)"
                % (elapsed_sec, self.limits.max_operator_response_sec)
            )
        return True, ""
