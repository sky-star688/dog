"""MyArm-2 ASCII action validation.

The vendor STM32 firmware accepts only ASCII frames starting with ``$``.
It provides no generic completion ACK for ``$DGS`` action groups, so a command
must never be reported as physically completed merely because bytes were
written.  Coordinates are rejected here: ``$KMS`` uses the arm-base frame in
millimetres while perception produces map-frame metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

ARM_ACTIONS = ("home", "stop", "pick", "place")
GROUP_ACTIONS = ("pick", "place")
GROUP_MIN = 0
GROUP_MAX = 9999
MAX_FRAME_BYTES = 64


def _valid_group(value) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and GROUP_MIN <= value <= GROUP_MAX
    )


def validate_ascii_frame(name, value) -> Optional[str]:
    if not isinstance(value, str) or not value:
        return "%s must be a non-empty ASCII frame" % name
    if len(value.encode("ascii", errors="ignore")) > MAX_FRAME_BYTES:
        return "%s exceeds %d bytes" % (name, MAX_FRAME_BYTES)
    if any(ord(char) > 127 for char in value):
        return "%s must contain only ASCII characters" % name
    if "\n" in value or "\r" in value:
        return "%s must not contain newlines" % name
    if not value.startswith("$") or not value.endswith("!"):
        return "%s must use the vendor '$...!' command format" % name
    return None


@dataclass(frozen=True)
class ArmConfig:
    pick_group: Optional[int] = None
    place_group: Optional[int] = None
    home_command: str = "$DJR!"
    stop_command: str = "$DST!"

    def validate(self) -> List[str]:
        errors = []
        for name, value in (
            ("home_command", self.home_command),
            ("stop_command", self.stop_command),
        ):
            error = validate_ascii_frame(name, value)
            if error:
                errors.append(error)
        for name, value in (("pick_group", self.pick_group), ("place_group", self.place_group)):
            if value is not None and not _valid_group(value):
                errors.append("%s must be an integer in [0, 9999]" % name)
        return errors

    def group_for(self, action: str) -> int:
        if action == "pick":
            group = self.pick_group
        elif action == "place":
            group = self.place_group
        else:
            raise ValueError("%s is not a stored-group action" % action)
        if group is None:
            raise ValueError(
                "%s_group is not configured; record and verify the action group first" % action
            )
        return group

    def missing_groups(self) -> List[str]:
        missing = []
        if self.pick_group is None:
            missing.append("pick_group")
        if self.place_group is None:
            missing.append("place_group")
        return missing


@dataclass(frozen=True)
class ArmCommand:
    action: str

    def validate(self, config: ArmConfig) -> List[str]:
        errors = []
        if self.action not in ARM_ACTIONS:
            return ["unsupported arm action: %s" % self.action]
        if self.action == "home":
            error = validate_ascii_frame("home_command", config.home_command)
            if error:
                errors.append(error)
        elif self.action == "stop":
            error = validate_ascii_frame("stop_command", config.stop_command)
            if error:
                errors.append(error)
        else:
            group = config.pick_group if self.action == "pick" else config.place_group
            if group is None:
                errors.append("%s_group is not configured" % self.action)
            elif not _valid_group(group):
                errors.append("%s_group must be an integer in [0, 9999]" % self.action)
        return errors

    def to_frame(self, config: ArmConfig) -> bytes:
        errors = self.validate(config)
        if errors:
            raise ValueError("; ".join(errors))
        if self.action == "home":
            frame = config.home_command
        elif self.action == "stop":
            frame = config.stop_command
        else:
            frame = "$DGS:%d!" % config.group_for(self.action)
        return frame.encode("ascii")


def encode_command(command: ArmCommand, config: ArmConfig) -> bytes:
    return command.to_frame(config)


def home_action() -> ArmCommand:
    return ArmCommand("home")


def stop_action() -> ArmCommand:
    return ArmCommand("stop")


def pick_action() -> ArmCommand:
    return ArmCommand("pick")


def place_action() -> ArmCommand:
    return ArmCommand("place")


def finite_seconds(value, name, minimum=0.0, maximum=60.0):
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or not minimum <= float(value) <= maximum
    ):
        raise ValueError("%s must be in [%.1f, %.1f]" % (name, minimum, maximum))
    return float(value)
