"""Tests for geometry helpers (no ROS deps)."""

import math

import pytest

from lite3_inspection.geometry import (
    bbox_center_region,
    deproject_pixel,
    quaternion_from_yaw,
    transform_point,
    yaw_from_quaternion,
)


def test_quaternion_roundtrip():
    q = quaternion_from_yaw(0.7)
    assert abs(yaw_from_quaternion(*q) - 0.7) < 1e-9
    with pytest.raises(ValueError):
        quaternion_from_yaw(math.nan)


def test_transform_translation_only():
    point = transform_point((1.0, 2.0, 3.0), (10.0, 0.0, -1.0), (0.0, 0.0, 0.0, 1.0))
    assert point == (11.0, 2.0, 2.0)


def test_transform_yaw_90():
    half = math.radians(90.0) * 0.5
    q = (0.0, 0.0, math.sin(half), math.cos(half))
    point = transform_point((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), q)
    assert abs(point[0]) < 1e-9 and abs(point[1] - 1.0) < 1e-9


def test_deproject_center():
    x, y, z = deproject_pixel(320.0, 240.0, 2.0, 600.0, 600.0, 320.0, 240.0)
    assert (x, y, z) == (0.0, 0.0, 2.0)


def test_deproject_rejects_bad():
    with pytest.raises(ValueError):
        deproject_pixel(0, 0, 0.0, 600.0, 600.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        deproject_pixel(math.nan, 0, 1.0, 600.0, 600.0, 0.0, 0.0)


def test_bbox_center_region():
    left, top, right, bottom = bbox_center_region((100, 100, 200, 200), 640, 480)
    assert left < 150 < right
    assert top < 150 < bottom
    assert 0 <= left and right <= 640


def test_bbox_invalid_area():
    with pytest.raises(ValueError):
        bbox_center_region((10, 10, 10, 20), 640, 480)
