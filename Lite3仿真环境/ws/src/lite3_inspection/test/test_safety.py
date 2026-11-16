"""Tests for safety limits and arbitration (no ROS deps)."""

import math

import pytest

from lite3_inspection.safety import (
    CommandArbiter,
    CommandSample,
    SafetyLimits,
    TwistValues,
    clamp_twist,
    twist_from_sequence,
)


class TestClamp:
    def test_within_limits_passes(self):
        result = clamp_twist(TwistValues(linear_x=0.2, angular_z=0.3), SafetyLimits())
        assert result.valid_input
        assert not result.clamped
        assert result.values.linear_x == 0.2

    def test_clamps_all_axes(self):
        result = clamp_twist(
            TwistValues(linear_x=9.0, linear_y=-9.0, angular_z=9.0), SafetyLimits()
        )
        assert result.clamped
        assert result.values.linear_x == 0.30
        assert result.values.linear_y == -0.15
        assert result.values.angular_z == 0.50

    def test_zero_limit_rejects_axis(self):
        result = clamp_twist(TwistValues(linear_z=0.1), SafetyLimits())
        assert result.values.linear_z == 0.0
        assert "linear_z" in result.violations

    def test_non_finite_rejected_whole(self):
        result = clamp_twist(TwistValues(linear_x=math.nan, linear_y=0.1), SafetyLimits())
        assert not result.valid_input
        assert result.values.as_tuple() == (0.0,) * 6

    def test_invalid_limits_raise(self):
        with pytest.raises(ValueError):
            clamp_twist(TwistValues(), SafetyLimits(max_linear_x=-1.0))


class TestArbiter:
    def _sample(self, source, priority, stamp, value=0.1):
        return CommandSample(source, priority, stamp, TwistValues(linear_x=value))

    def test_priority_wins(self):
        arbiter = CommandArbiter()
        now = 100.0
        selection = arbiter.select(
            now,
            [
                self._sample("nav", 10, now - 0.05),
                self._sample("teleop", 100, now - 0.05, 0.2),
                self._sample("aux", 20, now - 0.05, 0.3),
            ],
        )
        assert selection.active
        assert selection.source == "teleop"
        assert selection.values.linear_x == 0.2

    def test_stale_ignored(self):
        arbiter = CommandArbiter(default_timeout_sec=0.3)
        selection = arbiter.select(100.0, [self._sample("nav", 10, 90.0)])
        assert not selection.active
        assert selection.reason == "no_fresh_command"

    def test_estop_wins(self):
        arbiter = CommandArbiter()
        selection = arbiter.select(100.0, [self._sample("teleop", 100, 100.0)], estop=True)
        assert not selection.active
        assert selection.reason == "emergency_stop"

    def test_disabled_means_zero(self):
        arbiter = CommandArbiter()
        selection = arbiter.select(100.0, [self._sample("teleop", 100, 100.0)], enabled=False)
        assert not selection.active
        assert selection.values.is_zero()

    def test_invalid_sample_ignored(self):
        arbiter = CommandArbiter()
        sample = CommandSample("nav", 10, 100.0, TwistValues(), valid=False)
        selection = arbiter.select(100.0, [sample])
        assert not selection.active

    def test_clock_jump_ignored(self):
        arbiter = CommandArbiter()
        selection = arbiter.select(50.0, [self._sample("nav", 10, 100.0)])
        assert not selection.active


def test_twist_from_sequence():
    values = twist_from_sequence([1, 2, 3, 4, 5, 6])
    assert values.as_tuple() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    with pytest.raises(ValueError):
        twist_from_sequence([1, 2, 3])
