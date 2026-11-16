"""Tests for mission state machine (no ROS deps)."""

import pytest

from lite3_inspection.mission import (
    MissionConfig,
    MissionRunner,
    MissionState,
    Waypoint,
)


class FakeNavigation:
    def __init__(self):
        self.goal = None
        self.goal_id = None
        self.cancelled = False

    def send_goal(self, waypoint, goal_id):
        self.goal = waypoint
        self.goal_id = goal_id

    def cancel(self):
        self.cancelled = True


def _config(count=2, settle=0.0, retries=1):
    waypoints = [
        Waypoint("wp%d" % index, "map", float(index), 0.0, settle_sec=settle)
        for index in range(count)
    ]
    return MissionConfig("m1", "lite3_01", "lite3", "v1", "map", waypoints)


def _runner(clock=None):
    navigation = FakeNavigation()
    config = _config()
    runner = MissionRunner(config, navigation, clock=clock or (lambda: 0.0))
    return runner, navigation, config


def test_start_sends_first_goal():
    runner, navigation, _ = _runner()
    runner.start(now=0.0)
    assert runner.state == MissionState.RUNNING
    assert navigation.goal is not None
    assert navigation.goal.name == "wp0"


def test_success_advances_and_completes():
    runner, navigation, _ = _runner()
    runner.start(now=0.0)
    goal_id = navigation.goal_id
    runner.on_navigation_succeeded(1.0, goal_id)
    assert runner.state == MissionState.COMPLETED


def test_settle_then_complete():
    runner, navigation, _ = _runner()
    runner.config = MissionConfig(
        "m1", "lite3_01", "map", "v1", "map",
        [Waypoint("wp0", "map", 0.0, 0.0, settle_sec=2.0)],
    )
    runner.start(now=0.0)
    runner.on_navigation_succeeded(1.0, navigation.goal_id)
    assert runner.state == MissionState.SETTLING
    runner.tick(2.0)
    assert runner.state == MissionState.SETTLING
    runner.tick(3.5)
    assert runner.state == MissionState.COMPLETED


def test_failure_retries_then_fails():
    runner, navigation, _ = _runner()
    runner.start(now=0.0)
    runner.on_navigation_failed(1.0, navigation.goal_id, "boom")
    # one retry configured: still running on wp0
    assert runner.state == MissionState.RUNNING
    runner.on_navigation_failed(2.0, navigation.goal_id, "boom")
    assert runner.state == MissionState.FAILED


def test_pause_resume():
    runner, navigation, _ = _runner(clock=lambda: 5.0)
    runner.start(now=0.0)
    assert runner.pause(now=1.0)
    assert runner.state == MissionState.PAUSED
    assert navigation.cancelled
    assert runner.resume(now=2.0)
    assert runner.state == MissionState.RUNNING


def test_cancel():
    runner, _, _ = _runner()
    runner.start(now=0.0)
    assert runner.cancel(now=1.0)
    assert runner.state == MissionState.CANCELLED


def test_pause_from_idle_rejected():
    runner, _, _ = _runner()
    assert not runner.pause()


def test_config_from_dict():
    data = {
        "mission": {
            "mission_id": "m1",
            "robot_id": "lite3_01",
            "map_id": "map",
            "map_version": "v1",
            "frame_id": "map",
            "waypoints": [{"name": "wp0", "frame_id": "map", "x": 1.0, "y": 2.0}],
        }
    }
    config = MissionConfig.from_dict(data)
    assert len(config.waypoints) == 1
    with pytest.raises(ValueError):
        MissionConfig.from_dict({"mission": {"waypoints": []}})
