"""Read-only OpenMV target protocol and planar calibration helpers.

``@OVM:<cx>,<cy>,<width>,<height>,<pixels>!`` is a host-side frame, not a
vendor firmware frame.  OpenMV only reports a target bounding box; it never
writes to the arm.  A calibrated host may convert the pixel point to the
MyArm-2 base plane and construct a ``$KMS`` frame for the bench host tool.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

FRAME_PREFIX = "@OVM:"
FRAME_SUFFIX = "!"
MAX_FRAME_BYTES = 64
MAX_DIMENSION = 10000
MAX_PIXELS = 100000000
MAX_KMS_TIME_MS = 60000


@dataclass(frozen=True)
class OpenMVTarget:
    center_x: int
    center_y: int
    width: int
    height: int
    pixels: int

    def validate(self) -> List[str]:
        errors = []
        for name, value in (
            ("center_x", self.center_x),
            ("center_y", self.center_y),
            ("width", self.width),
            ("height", self.height),
            ("pixels", self.pixels),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                errors.append("%s must be an integer" % name)
            elif value < 0:
                errors.append("%s must not be negative" % name)
        if errors:
            return errors
        if self.width == 0 or self.height == 0:
            errors.append("width and height must be positive")
        if self.width > MAX_DIMENSION or self.height > MAX_DIMENSION:
            errors.append("bounding box exceeds %d pixels" % MAX_DIMENSION)
        if self.center_x > MAX_DIMENSION or self.center_y > MAX_DIMENSION:
            errors.append("center exceeds %d pixels" % MAX_DIMENSION)
        if self.pixels > MAX_PIXELS:
            errors.append("pixel count exceeds %d" % MAX_PIXELS)
        if self.pixels > self.width * self.height:
            errors.append("pixel count exceeds the bounding-box area")
        return errors

    def to_frame(self) -> bytes:
        errors = self.validate()
        if errors:
            raise ValueError("; ".join(errors))
        frame = "%s%d,%d,%d,%d,%d%s" % (
            FRAME_PREFIX, self.center_x, self.center_y,
            self.width, self.height, self.pixels, FRAME_SUFFIX,
        )
        return frame.encode("ascii")


def _coerce_ascii(value) -> bytes:
    if isinstance(value, bytes):
        data = value
    elif isinstance(value, str):
        try:
            data = value.encode("ascii")
        except UnicodeEncodeError:
            raise ValueError("OpenMV frame must be ASCII")
    else:
        raise ValueError("OpenMV frame must be bytes or str")
    data = data.strip()
    if not data:
        raise ValueError("OpenMV frame must not be empty")
    if len(data) > MAX_FRAME_BYTES:
        raise ValueError("OpenMV frame exceeds %d bytes" % MAX_FRAME_BYTES)
    if any(byte > 127 for byte in data):
        raise ValueError("OpenMV frame must be ASCII")
    return data


def parse_frame(value) -> OpenMVTarget:
    data = _coerce_ascii(value).strip()
    if not data.startswith(FRAME_PREFIX.encode("ascii")):
        raise ValueError("OpenMV frame must start with %s" % FRAME_PREFIX)
    if not data.endswith(FRAME_SUFFIX.encode("ascii")):
        raise ValueError("OpenMV frame must end with %s" % FRAME_SUFFIX)
    body = data[len(FRAME_PREFIX):-len(FRAME_SUFFIX)]
    if not body:
        raise ValueError("OpenMV frame has no fields")
    fields = body.split(b",")
    if len(fields) != 5:
        raise ValueError("OpenMV frame must have five integer fields")
    values = []
    for field in fields:
        if not field or field.strip() != field:
            raise ValueError("OpenMV frame fields must not contain whitespace")
        try:
            text = field.decode("ascii")
        except UnicodeDecodeError:
            raise ValueError("OpenMV frame fields must be ASCII")
        if text.startswith("+") or text.startswith("-"):
            raise ValueError("OpenMV frame fields must not be signed")
        if not text.isdigit():
            raise ValueError("OpenMV frame fields must be decimal integers")
        values.append(int(text))
    target = OpenMVTarget(*values)
    errors = target.validate()
    if errors:
        raise ValueError("; ".join(errors))
    return target


def extract_frames(data) -> Tuple[List[OpenMVTarget], List[str], bytes]:
    """Split an arbitrary byte stream into complete frames."""
    if isinstance(data, str):
        try:
            raw = data.encode("ascii")
        except UnicodeEncodeError:
            raise ValueError("OpenMV stream must be ASCII")
    elif isinstance(data, (bytes, bytearray)):
        raw = bytes(data)
    else:
        raise ValueError("OpenMV stream must be bytes or str")

    targets = []
    errors = []
    parts = raw.split(FRAME_SUFFIX.encode("ascii"))
    for part in parts[:-1]:
        try:
            targets.append(parse_frame(part + FRAME_SUFFIX.encode("ascii")))
        except ValueError as exc:
            errors.append(str(exc))
    remainder = parts[-1]
    if len(remainder) > MAX_FRAME_BYTES * 4:
        remainder = remainder[-(MAX_FRAME_BYTES * 4):]
    return targets, errors, remainder


def _solve_linear(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> List[float]:
    size = len(vector)
    if size == 0 or len(matrix) != size:
        raise ValueError("linear system must be square")
    augmented = []
    for row, value in zip(matrix, vector):
        if len(row) != size:
            raise ValueError("linear system must be square")
        augmented.append([float(item) for item in row] + [float(value)])
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("calibration points are degenerate")
        if pivot != column:
            augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for item in range(column, size + 1):
            augmented[column][item] /= divisor
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor == 0.0:
                continue
            for item in range(column, size + 1):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[row][size] for row in range(size)]


def _normalize_points(points: Sequence[Sequence[float]]):
    count = len(points)
    center_x = sum(float(point[0]) for point in points) / count
    center_y = sum(float(point[1]) for point in points) / count
    mean_distance = (
        sum(
            math.hypot(float(point[0]) - center_x, float(point[1]) - center_y)
            for point in points
        ) / count
    )
    if mean_distance < 1e-12:
        raise ValueError("calibration points are degenerate")
    scale = math.sqrt(2.0) / mean_distance
    normalized = [
        (scale * (float(point[0]) - center_x), scale * (float(point[1]) - center_y))
        for point in points
    ]
    transform = [
        [scale, 0.0, -scale * center_x],
        [0.0, scale, -scale * center_y],
        [0.0, 0.0, 1.0],
    ]
    return normalized, transform


def _matmul3(left, right):
    return [
        [
            sum(float(left[row][index]) * float(right[index][column]) for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _invert_similarity(transform):
    scale = float(transform[0][0])
    if abs(scale) < 1e-12:
        raise ValueError("calibration normalization is degenerate")
    translate_x = float(transform[0][2])
    translate_y = float(transform[1][2])
    return [
        [1.0 / scale, 0.0, -translate_x / scale],
        [0.0, 1.0 / scale, -translate_y / scale],
        [0.0, 0.0, 1.0],
    ]


def solve_homography(pixel_points, arm_points) -> List[List[float]]:
    """Fit a pixel -> arm-plane homography with at least four points."""
    if len(pixel_points) != len(arm_points):
        raise ValueError("pixel and arm point counts must match")
    if len(pixel_points) < 4:
        raise ValueError("at least four calibration points are required")

    raw_pixels = []
    raw_arm = []
    seen = set()
    for pixel, arm in zip(pixel_points, arm_points):
        if len(pixel) != 2 or len(arm) != 2:
            raise ValueError("calibration points must be 2D")
        u, v = float(pixel[0]), float(pixel[1])
        x, y = float(arm[0]), float(arm[1])
        if not all(math.isfinite(value) for value in (u, v, x, y)):
            raise ValueError("calibration points must be finite")
        key = (u, v)
        if key in seen:
            raise ValueError("calibration pixels must be unique")
        seen.add(key)
        raw_pixels.append((u, v))
        raw_arm.append((x, y))

    pixels, pixel_transform = _normalize_points(raw_pixels)
    targets, arm_transform = _normalize_points(raw_arm)

    normal = [[0.0] * 8 for _ in range(8)]
    target = [0.0] * 8
    for (u, v), (x, y) in zip(pixels, targets):
        rows = (
            (u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v, x),
            (0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v, y),
        )
        for row in rows:
            coefficients = row[:8]
            value = row[8]
            for left in range(8):
                target[left] += coefficients[left] * value
                for right in range(8):
                    normal[left][right] += coefficients[left] * coefficients[right]

    solution = _solve_linear(normal, target)
    normalized = [
        [solution[0], solution[1], solution[2]],
        [solution[3], solution[4], solution[5]],
        [solution[6], solution[7], 1.0],
    ]
    matrix = _matmul3(_invert_similarity(arm_transform), _matmul3(normalized, pixel_transform))
    divisor = matrix[2][2]
    if abs(divisor) < 1e-12:
        raise ValueError("calibration homography is degenerate")
    return [[value / divisor for value in row] for row in matrix]


def apply_homography(matrix, u: float, v: float) -> Tuple[float, float]:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("homography must be 3x3")
    values = [float(item) for row in matrix for item in row]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("homography must be finite")
    u = float(u)
    v = float(v)
    if not math.isfinite(u) or not math.isfinite(v):
        raise ValueError("pixel coordinates must be finite")
    denominator = values[6] * u + values[7] * v + values[8]
    if abs(denominator) < 1e-12:
        raise ValueError("homography is singular at this pixel")
    x = (values[0] * u + values[1] * v + values[2]) / denominator
    y = (values[3] * u + values[4] * v + values[5]) / denominator
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("homography produced a non-finite point")
    return x, y


def homography_error(matrix, pixel_points, arm_points) -> Tuple[float, float]:
    if len(pixel_points) != len(arm_points) or not pixel_points:
        raise ValueError("pixel and arm point counts must match")
    errors = []
    for pixel, arm in zip(pixel_points, arm_points):
        x, y = apply_homography(matrix, pixel[0], pixel[1])
        errors.append(math.hypot(x - float(arm[0]), y - float(arm[1])))
    rms = math.sqrt(sum(value * value for value in errors) / len(errors))
    return rms, max(errors)


@dataclass(frozen=True)
class KmsWorkspace:
    x_min_mm: float
    x_max_mm: float
    y_min_mm: float
    y_max_mm: float
    z_min_mm: float
    z_max_mm: float

    def validate(self) -> List[str]:
        errors = []
        for name, value in (
            ("x_min_mm", self.x_min_mm), ("x_max_mm", self.x_max_mm),
            ("y_min_mm", self.y_min_mm), ("y_max_mm", self.y_max_mm),
            ("z_min_mm", self.z_min_mm), ("z_max_mm", self.z_max_mm),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append("%s must be a number" % name)
            elif not math.isfinite(float(value)):
                errors.append("%s must be finite" % name)
        if errors:
            return errors
        if self.x_min_mm >= self.x_max_mm:
            errors.append("x_min_mm must be less than x_max_mm")
        if self.y_min_mm >= self.y_max_mm:
            errors.append("y_min_mm must be less than y_max_mm")
        if self.z_min_mm >= self.z_max_mm:
            errors.append("z_min_mm must be less than z_max_mm")
        return errors

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_min_mm <= x <= self.x_max_mm
            and self.y_min_mm <= y <= self.y_max_mm
            and self.z_min_mm <= z <= self.z_max_mm
        )


def load_calibration_file(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except Exception as exc:
        raise ValueError("cannot read calibration JSON: %s" % exc)
    if not isinstance(payload, dict):
        raise ValueError("calibration JSON must be an object")

    matrix = payload.get("homography")
    if not isinstance(matrix, list) or len(matrix) != 3:
        raise ValueError("homography must be a 3x3 array")
    for row in matrix:
        if not isinstance(row, list) or len(row) != 3:
            raise ValueError("homography must be a 3x3 array")
        for value in row:
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise ValueError("homography values must be finite numbers")

    workspace = None
    raw_workspace = payload.get("workspace_mm")
    if raw_workspace is not None:
        if not isinstance(raw_workspace, dict):
            raise ValueError("workspace_mm must be an object")
        try:
            workspace = KmsWorkspace(
                float(raw_workspace["x"][0]), float(raw_workspace["x"][1]),
                float(raw_workspace["y"][0]), float(raw_workspace["y"][1]),
                float(raw_workspace["z"][0]), float(raw_workspace["z"][1]),
            )
        except Exception as exc:
            raise ValueError("invalid workspace_mm: %s" % exc)
        errors = workspace.validate()
        if errors:
            raise ValueError("invalid workspace_mm: %s" % "; ".join(errors))
    return matrix, workspace


def build_kms_frame(x_mm, y_mm, z_mm, time_ms, workspace=None) -> bytes:
    values = (x_mm, y_mm, z_mm)
    if not all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        for value in values
    ):
        raise ValueError("KMS coordinates must be finite numbers")
    if isinstance(time_ms, bool) or not isinstance(time_ms, int):
        raise ValueError("KMS time must be an integer")
    if not 1 <= time_ms <= MAX_KMS_TIME_MS:
        raise ValueError("KMS time must be in [1, %d] ms" % MAX_KMS_TIME_MS)
    if workspace is not None:
        errors = workspace.validate()
        if errors:
            raise ValueError("; ".join(errors))
        if not workspace.contains(float(x_mm), float(y_mm), float(z_mm)):
            raise ValueError("KMS target is outside the configured workspace")
    frame = "$KMS:%.1f,%.1f,%.1f,%d!" % (float(x_mm), float(y_mm), float(z_mm), time_ms)
    if len(frame) > 64:
        raise ValueError("KMS frame exceeds 64 bytes")
    return frame.encode("ascii")


def target_to_kms(target, homography, z_mm, time_ms, workspace=None):
    errors = target.validate()
    if errors:
        raise ValueError("; ".join(errors))
    x, y = apply_homography(homography, target.center_x, target.center_y)
    return x, y, build_kms_frame(x, y, z_mm, time_ms, workspace)
