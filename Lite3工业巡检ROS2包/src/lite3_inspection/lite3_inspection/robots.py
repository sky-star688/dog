"""Robot identity, namespace and map metadata helpers for multi-robot use."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, List, Optional


_NAME = re.compile(r"^[a-z][a-z0-9_]{0,62}$")


def validate_robot_id(value: str) -> str:
    if not isinstance(value, str) or not _NAME.match(value):
        raise ValueError("robot_id must match [a-z][a-z0-9_]{0,62}, got %r" % (value,))
    return value


def normalize_namespace(value: Optional[str]) -> str:
    """Return a ROS namespace without a trailing slash (root = "")."""
    if value is None:
        return ""
    if not isinstance(value, str):
        raise ValueError("namespace must be a string or None")
    namespace = value.strip()
    if not namespace or namespace == "/":
        return ""
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    parts = [part for part in namespace.split("/") if part]
    for part in parts:
        if not _NAME.match(part):
            raise ValueError("invalid namespace segment: %r" % part)
    return "/" + "/".join(parts)


def topic(namespace: str, suffix: str) -> str:
    if not suffix or suffix.startswith("/"):
        raise ValueError("suffix must be a non-empty relative name")
    base = normalize_namespace(namespace)
    return "%s/%s" % (base, suffix) if base else "/" + suffix


@dataclass(frozen=True)
class RobotContext:
    robot_id: str
    namespace: str = ""
    site_id: str = "default"
    map_id: str = "lite3"
    map_version: str = "unversioned"

    def validate(self) -> List[str]:
        errors: List[str] = []
        try:
            validate_robot_id(self.robot_id)
        except ValueError as exc:
            errors.append(str(exc))
        try:
            normalize_namespace(self.namespace)
        except ValueError as exc:
            errors.append(str(exc))
        for name in ("site_id", "map_id", "map_version"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                errors.append("%s must be a non-empty string" % name)
        return errors

    @property
    def ns(self) -> str:
        return normalize_namespace(self.namespace)

    def topic(self, suffix: str) -> str:
        return topic(self.ns, suffix)

    def frame(self, suffix: str) -> str:
        if not suffix or suffix.startswith("/"):
            raise ValueError("frame suffix must be relative")
        return "%s_%s" % (self.robot_id, suffix)

    def to_dict(self) -> Dict[str, object]:
        errors = self.validate()
        if errors:
            raise ValueError("; ".join(errors))
        return {
            "robot_id": self.robot_id,
            "namespace": self.ns,
            "site_id": self.site_id,
            "map_id": self.map_id,
            "map_version": self.map_version,
        }
