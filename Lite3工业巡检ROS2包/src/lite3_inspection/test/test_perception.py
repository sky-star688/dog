"""Tests for perception protocol and projection (no ROS deps)."""

import pytest

from lite3_inspection.perception import (
    CameraIntrinsics,
    Detection,
    MapTransform,
    ProjectionError,
    detections_from_backend,
    project_detection,
)


def _detection(bbox=(100, 100, 200, 200)):
    return Detection(
        class_id=0,
        label="block",
        confidence=0.9,
        bbox=bbox,
        center_u=(bbox[0] + bbox[2]) * 0.5,
        center_v=(bbox[1] + bbox[3]) * 0.5,
    )


def _intrinsics():
    return CameraIntrinsics(640, 480, 600.0, 600.0, 320.0, 240.0)


def test_detection_validation():
    _detection().validate()
    with pytest.raises(ValueError):
        Detection(0, "", 0.5, (0, 0, 1, 1), 0.5, 0.5).validate()
    with pytest.raises(ValueError):
        Detection(0, "block", 2.0, (0, 0, 1, 1), 0.5, 0.5).validate()


def test_project_flat_depth():
    depth = lambda u, v: 2.0
    result = project_detection(_detection(), depth, _intrinsics())
    assert abs(result.depth_m - 2.0) < 1e-9
    assert result.point_map is None
    assert result.valid_ratio == 1.0


def test_project_with_transform():
    depth = lambda u, v: 2.0
    transform = MapTransform((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    result = project_detection(_detection(), depth, _intrinsics(), map_from_camera=transform)
    assert result.point_map is not None
    assert abs(result.point_map[2]) < 1e-9


def test_project_rejects_invalid_depth():
    with pytest.raises(ProjectionError):
        project_detection(_detection(), lambda u, v: 0.0, _intrinsics())
    with pytest.raises(ProjectionError):
        project_detection(_detection(), lambda u, v: 2.0, _intrinsics(), min_valid_ratio=1.1)


def test_project_rejects_unstable_depth():
    depth = lambda u, v: 2.0 if (u + v) % 2 else 5.0
    with pytest.raises(ProjectionError):
        project_detection(_detection(), depth, _intrinsics(), max_mad_m=0.01)


def test_project_out_of_range():
    with pytest.raises(ProjectionError):
        project_detection(_detection(), lambda u, v: 10.0, _intrinsics(), max_depth_m=6.0)


def test_detections_from_backend():
    rows = [[10, 10, 50, 50, 0.9, 0], [10, 10, 50, 50, 0.2, 0], [0, 0, 5, 5, 0.8, 9]]
    result = detections_from_backend(rows, ["block"], 640, 480, confidence_threshold=0.5)
    assert len(result) == 1
    assert result[0].label == "block"
