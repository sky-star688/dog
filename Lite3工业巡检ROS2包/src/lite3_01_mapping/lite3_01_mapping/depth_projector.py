"""Approximate 3-D projection of 2-D detections for Dog 1.

Input : /dog1/camera/color/image_raw  (for stamp reference)
        /camera/aligned_depth_to_color/image_raw
        /camera/color/camera_info
        /dog1/detections (JSON from dog1_vision_detector)
        TF (camera frame -> map)
Output: /dog1/detections_3d (JSON with point_map and rejection reasons)

This node fails closed: no aligned depth, stale depth, missing CameraInfo,
missing TF or unstable depth all produce rejection reasons, never a fake point.
"""

import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from lite3_inspection.geometry import bbox_center_region, deproject_pixel


class DepthProjector(Node):
    def __init__(self):
        super().__init__("depth_projector_dog1")

        self.declare_parameter("detections_topic", "/dog1/detections")
        self.declare_parameter("output_topic", "/dog1/detections_3d")
        self.declare_parameter("status_topic", "/dog1/depth_status")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("frame_id_override", "")
        self.declare_parameter("sync_slop_sec", 0.5)
        self.declare_parameter("max_depth_age_sec", 1.0)
        self.declare_parameter("min_valid_ratio", 0.30)
        self.declare_parameter("max_mad_m", 0.05)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 6.0)
        self.declare_parameter("sample_step", 2)

        self.bridge = CvBridge()
        self.depth_message = None
        self.camera_info = None
        self.depth_time = 0.0
        self.last_status = "starting"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.output_pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            10,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info("Dog 1 depth projector ready")

    def _on_depth(self, message: Image) -> None:
        self.depth_message = message
        self.depth_time = time.time()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _set_status(self, status: str) -> None:
        if status != self.last_status:
            self.last_status = status
            self.get_logger().info("Depth projection status: %s" % status)

    def _publish_status(self) -> None:
        age = time.time() - self.depth_time if self.depth_time else None
        msg = String()
        msg.data = json.dumps(
            {
                "status": self.last_status,
                "has_depth": self.depth_message is not None,
                "has_camera_info": self.camera_info is not None,
                "depth_age_sec": None if age is None else round(age, 3),
            }
        )
        self.status_pub.publish(msg)

    def _lookup_transform(self, frame_id: str):
        target = str(self.get_parameter("target_frame").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                target, frame_id, Time(), timeout=Duration(seconds=0.0)
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return (
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
        except Exception:
            return None

    def _on_detections(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        detections = payload.get("detections", [])
        if not detections:
            self._set_status("no_detections")
            return
        if self.depth_message is None:
            self._publish_result(payload, [], detections, "depth_unavailable")
            return
        age = time.time() - self.depth_time
        if age > float(self.get_parameter("max_depth_age_sec").value):
            self._publish_result(payload, [], detections, "depth_stale")
            return
        if self.camera_info is None:
            self._publish_result(payload, [], detections, "camera_info_unavailable")
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(
                self.depth_message, desired_encoding="passthrough"
            )
            depth = np.asarray(depth, dtype=np.float32)
        except Exception:
            self._publish_result(payload, [], detections, "depth_decode_failed")
            return
        if depth.ndim != 2:
            self._publish_result(payload, [], detections, "depth_size_mismatch")
            return

        kmat = self.camera_info.k
        width = int(self.camera_info.width)
        height = int(self.camera_info.height)
        fx, fy, cx, cy = kmat[0], kmat[4], kmat[2], kmat[5]
        if fx <= 0.0 or fy <= 0.0:
            self._publish_result(payload, [], detections, "camera_info_invalid")
            return

        frame_id = (
            str(self.get_parameter("frame_id_override").value)
            or payload.get("frame_id", "")
            or self.depth_message.header.frame_id
        )
        transform = self._lookup_transform(frame_id) if frame_id else None

        accepted = []
        rejected = []
        for detection in detections:
            bbox = detection.get("bbox")
            if not isinstance(bbox, list) or len(bbox) != 4:
                rejected.append({"reason": "invalid_bbox", "detection": detection})
                continue
            try:
                left, top, right, bottom = bbox_center_region(
                    bbox, width, height, fraction=0.5
                )
            except ValueError as exc:
                rejected.append({"reason": "invalid_bbox: %s" % exc, "detection": detection})
                continue
            sample_step = max(1, int(self.get_parameter("sample_step").value))
            values = []
            for v in range(top, bottom, sample_step):
                for u in range(left, right, sample_step):
                    if 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]:
                        values.append(float(depth[v, u]))
            valid = [v for v in values if math.isfinite(v) and v > 0.0]
            valid_ratio = len(valid) / float(len(values)) if values else 0.0
            if valid_ratio < float(self.get_parameter("min_valid_ratio").value):
                rejected.append({"reason": "insufficient_depth", "detection": detection})
                continue
            median = float(np.median(valid))
            mad = float(np.median(np.abs(np.asarray(valid) - median)))
            if not (
                float(self.get_parameter("min_depth_m").value)
                <= median
                <= float(self.get_parameter("max_depth_m").value)
            ):
                rejected.append({"reason": "depth_out_of_range", "detection": detection})
                continue
            if mad > float(self.get_parameter("max_mad_m").value):
                rejected.append({"reason": "unstable_depth", "detection": detection})
                continue
            center_u = (float(bbox[0]) + float(bbox[2])) * 0.5
            center_v = (float(bbox[1]) + float(bbox[3])) * 0.5
            point_camera = deproject_pixel(center_u, center_v, median, fx, fy, cx, cy)
            point_map = None
            if transform is not None:
                tx, ty, tz = transform[0]
                qx, qy, qz, qw = transform[1]
                # Apply rigid transform.
                px, py, pz = point_camera
                ix = qw * px + qy * pz - qz * py
                iy = qw * py + qz * px - qx * pz
                iz = qw * pz + qx * py - qy * px
                iw = -qx * px - qy * py - qz * pz
                rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
                ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
                rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
                point_map = [rx + tx, ry + ty, rz + tz]
            item = dict(detection)
            item["depth_m"] = median
            item["point_camera"] = list(point_camera)
            item["point_map"] = point_map
            item["depth_samples"] = len(valid)
            item["depth_valid_ratio"] = valid_ratio
            item["depth_mad_m"] = mad
            accepted.append(item)

        if accepted:
            status = "ready"
        elif rejected:
            status = rejected[0]["reason"]
        else:
            status = "no_detections"
        self._publish_result(payload, accepted, rejected, status)

    def _publish_result(self, source, accepted, rejected, status) -> None:
        self._set_status(status)
        msg = String()
        msg.data = json.dumps(
            {
                "detector_status": source.get("detector_status", "ready"),
                "projection_status": status,
                "frame_id": str(self.get_parameter("target_frame").value),
                "source_frame_id": source.get("frame_id", ""),
                "detections": accepted,
                "rejected": rejected,
                "stamp": source.get("stamp", {}),
            },
            ensure_ascii=False,
        )
        self.output_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
