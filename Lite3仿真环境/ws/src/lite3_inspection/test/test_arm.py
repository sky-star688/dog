"""Tests for MyArm ASCII protocol (no ROS deps)."""

import pytest

from lite3_inspection.arm import (
    ArmCommand,
    ArmConfig,
    encode_command,
    home_action,
    pick_action,
    place_action,
    stop_action,
)


def test_default_config_valid():
    assert ArmConfig().validate() == []


def test_frames():
    config = ArmConfig(pick_group=1, place_group=2)
    assert encode_command(home_action(), config) == b"$DJR!"
    assert encode_command(stop_action(), config) == b"$DST!"
    assert encode_command(pick_action(), config) == b"$DGS:1!"
    assert encode_command(place_action(), config) == b"$DGS:2!"


def test_unconfigured_group_rejected():
    with pytest.raises(ValueError):
        encode_command(pick_action(), ArmConfig())
    with pytest.raises(ValueError):
        encode_command(place_action(), ArmConfig(pick_group=1))


def test_unsupported_action_rejected():
    errors = ArmCommand("jump").validate(ArmConfig())
    assert errors


def test_invalid_frame_rejected():
    config = ArmConfig(home_command="DJR")
    assert config.validate()
    with pytest.raises(ValueError):
        encode_command(home_action(), config)


def test_group_bounds():
    assert ArmConfig().validate() == []
    assert ArmConfig(pick_group=10000).validate()
    assert ArmConfig(pick_group=-1).validate()
    assert ArmConfig(pick_group=True).validate()  # bool is not an int group


def test_stop_always_allowed_even_without_groups():
    assert encode_command(stop_action(), ArmConfig()) == b"$DST!"
