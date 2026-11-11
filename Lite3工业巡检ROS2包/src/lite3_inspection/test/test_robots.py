"""Tests for robot identity helpers (no ROS deps)."""

import pytest

from lite3_inspection.robots import (
    RobotContext,
    normalize_namespace,
    topic,
    validate_robot_id,
)


def test_validate_robot_id():
    assert validate_robot_id("lite3_01") == "lite3_01"
    for bad in ("", "Lite3", "1dog", "dog-1", "dog" * 20):
        with pytest.raises(ValueError):
            validate_robot_id(bad)


def test_normalize_namespace():
    assert normalize_namespace(None) == ""
    assert normalize_namespace("/") == ""
    assert normalize_namespace("dog1") == "/dog1"
    assert normalize_namespace("/dog1/") == "/dog1"
    with pytest.raises(ValueError):
        normalize_namespace("/Bad-Name")


def test_topic_join():
    assert topic("", "cmd_vel") == "/cmd_vel"
    assert topic("/dog1", "cmd_vel") == "/dog1/cmd_vel"
    with pytest.raises(ValueError):
        topic("", "/absolute")


def test_context_validate():
    assert RobotContext(robot_id="lite3_01").validate() == []
    errors = RobotContext(robot_id="Bad").validate()
    assert errors


def test_context_frame_and_dict():
    context = RobotContext(robot_id="lite3_01", namespace="/")
    assert context.frame("base") == "lite3_01_base"
    assert context.to_dict()["robot_id"] == "lite3_01"
