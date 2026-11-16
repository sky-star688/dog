"""Detection protocol and depth-based 3-D projection.

The detector backend is optional.  These types make it impossible for a node
to silently report detections when no model is loaded: callers must publish an
explicit DetectorStatus instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

from .geometry import (
    bbox_center_region,
    deproject_pixel,
    transform_point,
)


class DetectorStatus(str, Enum):
    READY = "ready"
    NO_FRAME = "no_frame"
    MODEL_MISSING = "model_missing"
    BACKEND_UNAVAILABLE = "backend_unavailable"
    INFERENCE_ERROR = "inference_error"


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    bbox: Tuple[float, float, float, float]
    center_u: float
    center_v: float

    def validate(self) -> None:
        if self.class_id < 0:
            raise ValueError("class_id must be non-negative")
        if not self.label:
            raise ValueError("label must not be empty")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if len(self.bbox) != 4 or not all(math.isfinite(v) for v in self.bbox):
            raise ValueError("bbox must contain four finite values")
        if not math.isfinite(self.center_u) or not math.isfinite(self.center_v):
            raise ValueError("center must be finite")

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "class_id": self.class_id,
            "label": self.label,
            "class_name": self.label,
            "confidence": self.confidence,
            "bbox": list(self.bbox),
            "center": [self.center_u, self.center_v],
        }


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        for name, value in (("fx", self.fx), ("fy", self.fy), ("cx", self.cx), ("cy", self.cy)):
            if not math.isfinite(value):
                raise ValueError("%s must be finite" % name)
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("fx and fy must be positive")
        if not (0.0 <= self.cx < self.width and 0.0 <= self.cy < self.height):
            raise ValueError("principal point must be inside the image")


@dataclass(frozen=True)
class MapTransform:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]

    def validate(self) -> None:
        if len(self.translation) != 3 or len(self.rotation) != 4:
            raise ValueError("translation needs 3 values and rotation 4")
        if not all(math.isfinite(v) for v in self.translation + self.rotation):
            raise ValueError("transform values must be finite")
        if all(abs(v) < 1e-12 for v in self.rotation):
            raise ValueError("rotation must be non-zero")


@dataclass(frozen=True)
class Detection3D:
    detection: Detection
    depth_m: float
    point_camera: Tuple[float, float, float]
    point_map: Optional[Tuple[float, float, float]]
    sample_count: int
    valid_ratio: float
    mad_m: float

    def to_dict(self) -> Dict[str, Any]:
        payload = self.detection.to_dict()
        payload.update(
            {
                "depth_m": self.depth_m,
                "point_camera": list(self.point_camera),
                "point_map": None if self.point_map is None else list(self.point_map),
                "depth_samples": self.sample_count,
                "depth_valid_ratio": self.valid_ratio,
                "depth_mad_m": self.mad_m,
            }
        )
        return payload


class ProjectionError(Exception):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def project_detection(
    detection: Detection,
    depth_at: Callable[[int, int], float],
    intrinsics: CameraIntrinsics,
    map_from_camera: Optional[MapTransform] = None,
    min_valid_ratio: float = 0.30,
    max_mad_m: float = 0.05,
    min_depth_m: float = 0.15,
    max_depth_m: float = 6.0,
    sample_step: int = 2,
) -> Detection3D:
    """Project a 2-D box to camera/map point using a robust depth median."""
    detection.validate()
    intrinsics.validate()
    if not 0.0 < min_valid_ratio <= 1.0:
        raise ValueError("min_valid_ratio must be in (0, 1]")
    if max_mad_m < 0.0 or not math.isfinite(max_mad_m):
        raise ValueError("max_mad_m must be non-negative")
    if sample_step <= 0:
        raise ValueError("sample_step must be positive")
    if map_from_camera is not None:
        map_from_camera.validate()

    try:
        left, top, right, bottom = bbox_center_region(
            detection.bbox, intrinsics.width, intrinsics.height, fraction=0.5
        )
    except ValueError as exc:
        raise ProjectionError("invalid_bbox: %s" % exc)

    values = []
    for v in range(top, bottom, sample_step):
        for u in range(left, right, sample_step):
            try:
                values.append(float(depth_at(u, v)))
            except (TypeError, ValueError):
                values.append(0.0)

    valid = [value for value in values if math.isfinite(value) and value > 0.0]
    if not values or not valid:
        raise ProjectionError("insufficient_depth")
    valid_ratio = len(valid) / float(len(values))
    if valid_ratio < min_valid_ratio:
        raise ProjectionError("insufficient_depth")

    valid_sorted = sorted(valid)
    median = valid_sorted[len(valid_sorted) // 2]
    deviations = sorted(abs(value - median) for value in valid)
    mad = deviations[len(deviations) // 2]

    if not min_depth_m <= median <= max_depth_m:
        raise ProjectionError("depth_out_of_range")
    if mad > max_mad_m:
        raise ProjectionError("unstable_depth")

    center_u = (detection.bbox[0] + detection.bbox[2]) * 0.5
    center_v = (detection.bbox[1] + detection.bbox[3]) * 0.5
    point_camera = deproject_pixel(
        center_u, center_v, median, intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy
    )
    point_map = None
    if map_from_camera is not None:
        point_map = transform_point(
            point_camera, map_from_camera.translation, map_from_camera.rotation
        )
    return Detection3D(
        detection=detection,
        depth_m=median,
        point_camera=point_camera,
        point_map=point_map,
        sample_count=len(valid),
        valid_ratio=valid_ratio,
        mad_m=mad,
    )


def detections_from_backend(
    rows: Sequence[Sequence[float]],
    labels: Sequence[str],
    width: int,
    height: int,
    confidence_threshold: float = 0.25,
) -> list:
    """Convert raw YOLO rows ``[x1, y1, x2, y2, confidence, class_id]``."""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if not labels:
        raise ValueError("labels must not be empty")
    detections = []
    for row in rows:
        if len(row) != 6:
            raise ValueError("each detection row must have six values")
        x1, y1, x2, y2, score, class_value = (float(v) for v in row)
        class_id = int(class_value)
        if class_id < 0 or class_id >= len(labels):
            continue
        if not 0.0 <= score <= 1.0 or score < confidence_threshold:
            continue
        x1 = max(0.0, min(float(width - 1), x1))
        x2 = max(0.0, min(float(width - 1), x2))
        y1 = max(0.0, min(float(height - 1), y1))
        y2 = max(0.0, min(float(height - 1), y2))
        if x2 <= x1 or y2 <= y1:
            continue
        detection = Detection(
            class_id=class_id,
            label=str(labels[class_id]),
            confidence=score,
            bbox=(x1, y1, x2, y2),
            center_u=(x1 + x2) * 0.5,
            center_v=(y1 + y2) * 0.5,
        )
        detection.validate()
        detections.append(detection)
    return detections
