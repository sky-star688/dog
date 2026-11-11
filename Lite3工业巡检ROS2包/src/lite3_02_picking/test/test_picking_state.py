"""Tests for Dog 2 picking domain logic."""

import pytest

from lite3_02_picking.picking_state import PickingModel, PickingState, TargetAssignment


def _assignment(**kwargs):
    values = dict(
        handoff_id="h1",
        target_id="t1",
        class_name="block",
        confidence=0.9,
        coarse_x=5.0,
        coarse_y=5.0,
        approach_x=4.0,
        approach_y=5.0,
        approach_yaw=0.0,
        centre_x=5.0,
        centre_y=5.0,
        retreat_x=3.0,
        retreat_y=5.0,
        retreat_yaw=3.14,
        workspace_id="ws_01",
    )
    values.update(kwargs)
    return TargetAssignment(**values)


class TestPickingModel:
    def test_initial_state(self):
        model = PickingModel(robot_id="lite3_02")
        assert model.state == PickingState.IDLE
        assert not model.map_ready
        assert not model.localization_ok
        assert model.assignment is None

    def test_assign(self):
        model = PickingModel()
        model.assign(_assignment())
        assert model.state == PickingState.ASSIGNED
        assert model.assignment.target_id == "t1"

    def test_full_lifecycle(self):
        model = PickingModel()
        model.assign(_assignment())
        for state in (
            PickingState.APPROACHING,
            PickingState.POSITIONING,
            PickingState.GRASPING,
            PickingState.PLACING,
            PickingState.RETREATING,
            PickingState.COMPLETED,
        ):
            model.state = state
        assert model.is_terminal

    def test_reset(self):
        model = PickingModel()
        model.assign(_assignment())
        model.reset()
        assert model.state == PickingState.IDLE
        assert model.assignment is None

    def test_assignment_validation(self):
        assert _assignment(handoff_id="").validate()
        assert not _assignment().validate()

    def test_to_dict(self):
        payload = PickingModel(robot_id="lite3_02").to_dict()
        assert payload["robot_id"] == "lite3_02"
        assert payload["state"] == "idle"
