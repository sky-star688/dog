"""Tests for event store (no ROS deps)."""

import json
from pathlib import Path

import pytest

from lite3_inspection.events import EventStore


def test_run_dir_and_events(tmp_path):
    store = EventStore(tmp_path, "lite3_01", "mission_1")
    assert store.run_dir.is_dir()
    store.append("custom_event", value=42)
    store.close("finished")

    lines = store.events_path.read_text(encoding="utf-8").strip().splitlines()
    records = [json.loads(line) for line in lines]
    assert records[0]["type"] == "run_started"
    assert records[-1]["type"] == "run_finished"
    assert any(record.get("type") == "custom_event" for record in records)


def test_artifact_copy_and_hash(tmp_path):
    source = tmp_path / "shot.png"
    source.write_bytes(b"fake-image-bytes")
    store = EventStore(tmp_path / "runs", "lite3_01", "m1")
    record = store.add_artifact(source, "image")
    assert record["sha256"]
    assert (store.run_dir / record["path"]).is_file()
    store.close()


def test_missing_artifact_raises(tmp_path):
    store = EventStore(tmp_path / "runs", "lite3_01", "m1")
    with pytest.raises(FileNotFoundError):
        store.add_artifact(tmp_path / "missing.png", "image")
    store.close()


def test_bad_robot_id_rejected(tmp_path):
    with pytest.raises(ValueError):
        EventStore(tmp_path, "../evil", "m1")


def test_append_after_close_raises(tmp_path):
    store = EventStore(tmp_path / "runs", "lite3_01", "m1")
    store.close()
    with pytest.raises(RuntimeError):
        store.append("late")
