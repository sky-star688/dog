"""Append-only JSONL event storage for inspection runs.

Events are written with fsync so a power loss does not silently discard the
last record.  Artifacts are copied into the run directory before being
referenced; callers must pass the original path, never a fabricated one.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


_SAFE_SEGMENT = re.compile(r"^[A-Za-z0-9_.-]+$")


def _safe_segment(value: str, name: str) -> str:
    if not isinstance(value, str) or not _SAFE_SEGMENT.match(value):
        raise ValueError("%s contains unsupported characters: %r" % (name, value))
    if value in (".", ".."):
        raise ValueError("%s must not be a relative path segment" % name)
    return value


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class EventStore:
    """Create a run directory and append validated JSON records."""

    def __init__(self, root: Path, robot_id: str, mission_id: str) -> None:
        self.root = Path(root)
        self.robot_id = _safe_segment(robot_id, "robot_id")
        self.mission_id = _safe_segment(mission_id, "mission_id")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self.run_id = "%s_%s_%s" % (self.robot_id, stamp, self.mission_id)
        self.run_dir = self.root / self.robot_id / self.run_id
        self.artifact_dir = self.run_dir / "artifacts"
        self.events_path = self.run_dir / "events.jsonl"
        self._sequence = 0
        self._closed = False
        self.artifact_dir.mkdir(parents=True, exist_ok=False)
        self._write(
            {
                "type": "run_started",
                "robot_id": self.robot_id,
                "mission_id": self.mission_id,
                "run_id": self.run_id,
            }
        )

    def _write(self, payload: Dict[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("event store is closed")
        record = dict(payload)
        record.setdefault("ts", _utc_now())
        record["seq"] = self._sequence
        self._sequence += 1
        line = json.dumps(record, ensure_ascii=False, sort_keys=True)
        with open(self.events_path, "a", encoding="utf-8") as handle:
            handle.write(line + "\n")
            handle.flush()
            os.fsync(handle.fileno())

    def append(self, event_type: str, **fields: Any) -> Dict[str, Any]:
        if not event_type or not isinstance(event_type, str):
            raise ValueError("event_type must be a non-empty string")
        payload = {"type": event_type, "robot_id": self.robot_id}
        payload.update(fields)
        self._write(payload)
        return payload

    def add_artifact(self, source: Path, artifact_type: str, metadata: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        source = Path(source)
        if not source.is_file():
            raise FileNotFoundError(str(source))
        artifact_type = _safe_segment(artifact_type, "artifact_type")
        digest = hashlib.sha256()
        with open(source, "rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        suffix = source.suffix
        target = self.artifact_dir / (digest.hexdigest() + suffix)
        if not target.exists():
            shutil.copy2(str(source), str(target))
        record = {
            "type": "artifact",
            "artifact_type": artifact_type,
            "path": str(target.relative_to(self.run_dir)),
            "sha256": digest.hexdigest(),
            "bytes": target.stat().st_size,
            "source_name": source.name,
        }
        if metadata:
            record["metadata"] = metadata
        self._write(record)
        return record

    def close(self, status: str = "finished") -> None:
        if self._closed:
            return
        self._write({"type": "run_finished", "status": status})
        self._closed = True

    def __enter__(self) -> "EventStore":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if not self._closed:
            self.close("finished" if exc_type is None else "error")
