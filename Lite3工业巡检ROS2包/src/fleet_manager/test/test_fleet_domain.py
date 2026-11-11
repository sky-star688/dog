"""Tests for fleet_manager domain logic (no ROS dependencies)."""

import time

import pytest

from fleet_manager.fleet_state import (
    FleetModel,
    FleetState,
    RobotRecord,
    RobotRole,
    RobotStatus,
)
from fleet_manager.handoff import (
    HandoffError,
    HandoffPhase as HPhase,
    HandoffState,
    HandoffTarget,
)
from fleet_manager.workspace import (
    WorkspaceGeometry,
    WorkspaceManager,
    WorkspaceState,
)
from fleet_manager.fleet_safety import FleetSafetyChecker


def _target(**kwargs):
    values = dict(
        target_id="t1",
        class_name="block",
        confidence=0.9,
        coarse_x=5.0,
        coarse_y=5.0,
        coarse_z=0.0,
        coarse_yaw=0.0,
        inspector_id="lite3_01",
    )
    values.update(kwargs)
    return HandoffTarget(**values)


class TestFleetModel:
    def test_register_robots(self):
        model = FleetModel()
        model.register_robot("lite3_01", RobotRole.INSPECTOR)
        model.register_robot("lite3_02", RobotRole.OPERATOR)
        assert model.inspector.robot_id == "lite3_01"
        assert model.operator.robot_id == "lite3_02"

    def test_any_estop(self):
        model = FleetModel()
        model.register_robot("lite3_01", RobotRole.INSPECTOR)
        model.register_robot("lite3_02", RobotRole.OPERATOR)
        assert not model.any_estop
        model.robots["lite3_01"].estop = True
        assert model.any_estop

    def test_can_start_handoff(self):
        model = FleetModel()
        model.register_robot("lite3_01", RobotRole.INSPECTOR)
        model.register_robot("lite3_02", RobotRole.OPERATOR)
        model.state = FleetState.READY
        for robot in model.robots.values():
            robot.last_heartbeat = time.monotonic()
            robot.localization_ok = True
        ok, reason = model.can_start_handoff("lite3_01", "lite3_02", "ws_01")
        assert ok, reason

    def test_cannot_start_when_operator_busy(self):
        model = FleetModel()
        model.register_robot("lite3_01", RobotRole.INSPECTOR)
        model.register_robot("lite3_02", RobotRole.OPERATOR)
        model.state = FleetState.READY
        for robot in model.robots.values():
            robot.last_heartbeat = time.monotonic()
            robot.localization_ok = True
        model.robots["lite3_02"].status = RobotStatus.APPROACHING
        ok, reason = model.can_start_handoff("lite3_01", "lite3_02", "ws_01")
        assert not ok
        assert "not_idle" in reason

    def test_emergency_and_clear(self):
        model = FleetModel()
        model.register_robot("lite3_01", RobotRole.INSPECTOR)
        model.emergency_stop()
        assert model.state == FleetState.EMERGENCY
        model.clear_emergency()
        assert model.state == FleetState.READY


class TestHandoffState:
    def test_lifecycle(self):
        state = HandoffState.create(_target(), "lite3_01", "lite3_02", "ws_01")
        assert state.phase == HPhase.REQUESTED
        state.accept()
        assert state.phase == HPhase.ACCEPTED
        for phase in (
            HPhase.APPROACHING,
            HPhase.POSITIONING,
            HPhase.GRASPING,
            HPhase.PLACING,
            HPhase.RETREATING,
        ):
            state.transition_to(phase)
        state.complete()
        assert state.phase == HPhase.COMPLETED
        assert state.is_terminal

    def test_invalid_transition(self):
        state = HandoffState.create(_target(), "lite3_01", "lite3_02", "ws_01")
        with pytest.raises(HandoffError):
            state.transition_to(HPhase.COMPLETED)

    def test_fail(self):
        state = HandoffState.create(_target(), "lite3_01", "lite3_02", "ws_01")
        state.accept()
        state.transition_to(HPhase.APPROACHING)
        state.fail("navigation error")
        assert state.phase == HPhase.FAILED
        assert state.error_message == "navigation error"

    def test_target_validation(self):
        assert _target(target_id="").validate()
        assert _target(confidence=2.0).validate()
        assert not _target().validate()

    def test_fine_position(self):
        state = HandoffState.create(_target(), "lite3_01", "lite3_02", "ws_01")
        state.set_fine_position(100.0, 50.0, 200.0)
        assert state.fine_valid
        assert state.fine_x == 100.0


class TestWorkspaceManager:
    def _geometry(self, ws_id="ws_01"):
        return WorkspaceGeometry(
            workspace_id=ws_id,
            approach_x=4.0,
            approach_y=5.0,
            approach_yaw=0.0,
            centre_x=5.0,
            centre_y=5.0,
            retreat_x=4.0,
            retreat_y=5.0,
            retreat_yaw=3.14,
        )

    def test_reserve_and_release(self):
        manager = WorkspaceManager()
        manager.register_workspace(self._geometry())
        assert manager.is_workspace_free("ws_01")
        reservation = manager.reserve("ws_01", "lite3_02", "t1", duration_sec=60.0)
        assert reservation.state == WorkspaceState.RESERVED
        assert not manager.is_workspace_free("ws_01")
        manager.occupy("ws_01", "lite3_02")
        manager.release("ws_01", "lite3_02")
        assert manager.is_workspace_free("ws_01")

    def test_cannot_double_reserve(self):
        manager = WorkspaceManager()
        manager.register_workspace(self._geometry())
        manager.reserve("ws_01", "lite3_02", "t1")
        with pytest.raises(ValueError):
            manager.reserve("ws_01", "lite3_01", "t2")

    def test_expired_release(self):
        manager = WorkspaceManager()
        manager.register_workspace(self._geometry())
        manager.reserve("ws_01", "lite3_02", "t1", duration_sec=0.001)
        time.sleep(0.01)
        released = manager.force_release_expired()
        assert "ws_01" in released
        assert manager.is_workspace_free("ws_01")


class TestFleetSafety:
    def test_workspace_intrusion(self):
        checker = FleetSafetyChecker()
        ok, _ = checker.check_workspace_intrusion(0.0, 0.0, 10.0, 10.0, "inspector", "occupied")
        assert ok
        ok, _ = checker.check_workspace_intrusion(8.0, 8.0, 10.0, 10.0, "inspector", "occupied")
        assert not ok

    def test_robot_separation(self):
        checker = FleetSafetyChecker()
        assert checker.check_robot_separation(0, 0, "r1", 10, 10, "r2")[0]
        assert not checker.check_robot_separation(0, 0, "r1", 1, 1, "r2")[0]

    def test_battery(self):
        checker = FleetSafetyChecker()
        assert checker.check_battery(23.0, "r1")[0]
        ok, _, action = checker.check_battery(21.5, "r1")
        assert not ok and action == "return_to_base"
        ok, _, action = checker.check_battery(20.0, "r1")
        assert not ok and action == "emergency_stop"


class TestRobotRecord:
    def test_online_and_accept(self):
        record = RobotRecord(robot_id="lite3_02", role=RobotRole.OPERATOR)
        record.last_heartbeat = time.monotonic()
        record.localization_ok = True
        record.status = RobotStatus.IDLE
        assert record.online
        assert record.can_accept_handoff

    def test_busy_rejects(self):
        record = RobotRecord(robot_id="lite3_02", role=RobotRole.OPERATOR)
        record.last_heartbeat = time.monotonic()
        record.localization_ok = True
        record.status = RobotStatus.APPROACHING
        assert not record.can_accept_handoff
