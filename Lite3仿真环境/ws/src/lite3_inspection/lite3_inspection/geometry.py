"""Pure geometry helpers (no ROS dependency)."""

from __future__ import annotations

import math
from typing import Sequence, Tuple


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation about +Z."""
    if not math.isfinite(float(yaw)):
        raise ValueError("yaw must be finite")
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the yaw component of a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_point(
    point: Sequence[float],
    translation: Sequence[float],
    rotation: Sequence[float],
) -> Tuple[float, float, float]:
    """Apply a rigid transform (translation + unit quaternion) to a point."""
    if len(point) != 3 or len(translation) != 3 or len(rotation) != 4:
        raise ValueError("point/translation need 3 values, rotation needs 4")
    px, py, pz = (float(v) for v in point)
    tx, ty, tz = (float(v) for v in translation)
    qx, qy, qz, qw = (float(v) for v in rotation)

    ix = qw * px + qy * pz - qz * py
    iy = qw * py + qz * px - qx * pz
    iz = qw * pz + qx * py - qy * px
    iw = -qx * px - qy * py - qz * pz

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx

    return rx + tx, ry + ty, rz + tz


def deproject_pixel(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """Pinhole deprojection: pixel + depth -> camera-frame point."""
    for name, value in (("u", u), ("v", v), ("depth_m", depth_m)):
        if not math.isfinite(float(value)):
            raise ValueError("%s must be finite" % name)
    if depth_m <= 0.0:
        raise ValueError("depth_m must be positive")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("focal length must be positive")
    x = (float(u) - float(cx)) / float(fx) * depth_m
    y = (float(v) - float(cy)) / float(fy) * depth_m
    return x, y, float(depth_m)


def bbox_center_region(
    bbox: Sequence[float],
    width: int,
    height: int,
    fraction: float = 0.5,
) -> Tuple[int, int, int, int]:
    """Return the central sub-region of a bbox as integer pixel bounds."""
    if len(bbox) != 4:
        raise ValueError("bbox must have four values")
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if not 0.0 < fraction <= 1.0:
        raise ValueError("fraction must be in (0, 1]")
    x1, y1, x2, y2 = (float(v) for v in bbox)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox must have positive area")
    center_x = (x1 + x2) * 0.5
    center_y = (y1 + y2) * 0.5
    half_w = (x2 - x1) * fraction * 0.5
    half_h = (y2 - y1) * fraction * 0.5
    left = max(0, int(center_x - half_w))
    top = max(0, int(center_y - half_h))
    right = min(width, int(center_x + half_w) + 1)
    bottom = min(height, int(center_y + half_h) + 1)
    if right <= left or bottom <= top:
        raise ValueError("central bbox region is empty")
    return left, top, right, bottom


def is_finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))
