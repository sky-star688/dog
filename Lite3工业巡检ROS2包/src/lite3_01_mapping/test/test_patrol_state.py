"""Tests for Dog 1 patrol domain logic."""

import pytest

from lite3_01_mapping.patrol_state import PatrolModel, PatrolState, PatrolTarget, PatrolWaypoint


class TestPatrolModel:
    def test_initial_state(self):
        model = PatrolModel(robot_id="lite3_01")
        assert model.state == PatrolState.IDLE
        assert not model.map_ready

    def test_set_waypoints(self):
        model = PatrolModel()
        model.set_waypoints([PatrolWaypoint("wp1", "map", 1.0, 2.0)])
        assert len(model.waypoints) == 1
        assert model.current_waypoint.name == "wp1"

    def test_set_waypoints_rejects_bad(self):
        model = PatrolModel()
        with pytest.raises(ValueError):
            model.set_waypoints([PatrolWaypoint("", "map", 1.0, 2.0)])

    def test_patrol_complete(self):
        model = PatrolModel()
        model.set_waypoints([PatrolWaypoint("wp1", "map", 1.0, 2.0)])
        model.current_index = 1
        assert model.patrol_complete

    def test_set_target(self):
        model = PatrolModel()
        model.set_target("block", 0.9, 5.0, 5.0, 0.0)
        assert model.state == PatrolState.TARGET_FOUND
        assert model.current_target.class_name == "block"
        assert model.pending_target

    def test_clear_target(self):
        model = PatrolModel()
        model.set_target("block", 0.9, 5.0, 5.0)
        model.clear_target()
        assert model.current_target is None
        assert not model.pending_target

    def test_target_to_dict(self):
        target = PatrolTarget(class_name="block", confidence=0.92, x=5.0, y=5.0)
        payload = target.to_dict()
        assert payload["class_name"] == "block"
        assert payload["requires_operation"] is True
