"""Tests for OpenMV protocol and calibration (no ROS deps)."""

import pytest

from lite3_inspection.openmv import (
    KmsWorkspace,
    OpenMVTarget,
    apply_homography,
    build_kms_frame,
    extract_frames,
    homography_error,
    parse_frame,
    solve_homography,
    target_to_kms,
)


def test_parse_valid_frame():
    target = parse_frame("@OVM:100,80,40,30,900!")
    assert target.center_x == 100
    assert target.width == 40


def test_parse_rejects_bad_frames():
    for frame in ("OVM:1,2,3,4,5!", "@OVM:1,2,3,4!", "@OVM:-1,2,3,4,5!",
                  "@OVM:1,2,3,4,5"):
        with pytest.raises(ValueError):
            parse_frame(frame)


def test_extract_frames_stream():
    targets, errors, remainder = extract_frames(
        b"@OVM:1,2,3,4,5!@OVM:6,7,8,9,10!partial"
    )
    assert len(targets) == 2
    assert not errors
    assert remainder == b"partial"


def test_target_validation():
    assert OpenMVTarget(10, 10, 5, 5, 20).validate() == []
    assert OpenMVTarget(10, 10, 0, 5, 0).validate()
    assert OpenMVTarget(10, 10, 5, 5, 100).validate()  # pixels > area


def test_homography_identity():
    points = [(0, 0), (100, 0), (100, 100), (0, 100)]
    matrix = solve_homography(points, points)
    x, y = apply_homography(matrix, 50, 50)
    assert abs(x - 50) < 1e-6 and abs(y - 50) < 1e-6
    rms, worst = homography_error(matrix, points, points)
    assert rms < 1e-6


def test_homography_needs_four_points():
    with pytest.raises(ValueError):
        solve_homography([(0, 0), (1, 1), (2, 2)], [(0, 0), (1, 1), (2, 2)])


def test_kms_frame():
    frame = build_kms_frame(100.0, 50.0, 40.0, 500)
    assert frame == b"$KMS:100.0,50.0,40.0,500!"


def test_kms_workspace_bounds():
    workspace = KmsWorkspace(0, 200, 0, 200, 0, 100)
    build_kms_frame(100, 100, 50, 500, workspace=workspace)
    with pytest.raises(ValueError):
        build_kms_frame(300, 100, 50, 500, workspace=workspace)


def test_target_to_kms():
    points = [(0, 0), (100, 0), (100, 100), (0, 100)]
    matrix = solve_homography(points, points)
    target = OpenMVTarget(50, 50, 10, 10, 100)
    x, y, frame = target_to_kms(target, matrix, 40.0, 500)
    assert abs(x - 50) < 1e-6
    assert frame.startswith(b"$KMS:")
