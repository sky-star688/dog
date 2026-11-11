"""Project 2-D detections to 3-D points using aligned depth and TF.

Shared implementation used by the standalone single-dog inspection stack.
The dual-dog Dog 1 package has its own equivalent node (dog1_depth_projector)
with Dog 1 topic defaults; both fail closed on missing/stale depth or TF.
"""

import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .geometry import bbox_center_region, deproject_pixel


class DepthProjector(Node):
    def __init__(self):
        super().__init__("depth_projector")

        self.declare_parameter("detections_topic", "/inspection/detections")
        self.declare_parameter("output_topic", "/inspection/detections_3d")
        self.declare_parameter("status_topic", "/inspection/depth_status")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("max_depth_age_sec", 1.0)
        self.declare_parameter("min_valid_ratio", 0.30)
        self.declare_parameter("max_mad_m", 0.05)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 6.0)
        self.declare_parameter("sample_step", 2)

        self.bridge = CvBridge()
        self._depth = None
        self._camera_info = None
        self._depth_time = 0.0
        self._last_status = "starting"
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.output_pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            String, str(self.get_parameter("detections_topic").value), self._on_detections, 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._on_depth, 10
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_camera_info, 10
        )
        self.create_timer(1.0, self._publish_status)

    def _on_depth(self, message):
        self._depth = message
        self._depth_time = time.time()

    def _on_camera_info(self, message):
        self._camera_info = message

    def _set_status(self, status):
        if status != self._last_status:
            self._last_status = status
            self.get_logger().info("Depth projection status: %s" % status)

    def _publish_status(self):
        age = time.time() - self._depth_time if self._depth_time else None
        msg = String()
        msg.data = json.dumps(
            {
                "status": self._last_status,
                "has_depth": self._depth is not None,
                "has_camera_info": self._camera_info is not None,
                "depth_age_sec": None if age is None else round(age, 3),
            }
        )
        self.status_pub.publish(msg)

    def _lookup_transform(self, frame_id):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("target_frame").value),
                frame_id,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)
        except Exception:
            return None

    def _publish_result(self, source, accepted, rejected, status):
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

    def _on_detections(self, message):
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        detections = payload.get("detections", [])
        if not detections:
            self._set_status("no_detections")
            return
        if self._depth is None:
            self._publish_result(payload, [], detections, "depth_unavailable")
            return
        if time.time() - self._depth_time > float(
            self.get_parameter("max_depth_age_sec").value
        ):
            self._publish_result(payload, [], detections, "depth_stale")
            return
        if self._camera_info is None:
            self._publish_result(payload, [], detections, "camera_info_unavailable")
            return
        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(self._depth, desired_encoding="passthrough"),
                dtype=np.float32,
            )
        except Exception:
            self._publish_result(payload, [], detections, "depth_decode_failed")
            return
        if depth.ndim != 2:
            self._publish_result(payload, [], detections, "depth_size_mismatch")
            return

        k = self._camera_info.k
        width = int(self._camera_info.width)
        height = int(self._camera_info.height)
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        frame_id = payload.get("frame_id", "") or self._depth.header.frame_id
        transform = self._lookup_transform(frame_id) if frame_id else None
        sample_step = max(1, int(self.get_parameter("sample_step").value))

        accepted = []
        rejected = []
        for detection in detections:
            bbox = detection.get("bbox")
            if not isinstance(bbox, list) or len(bbox) != 4:
                rejected.append({"reason": "invalid_bbox", "detection": detection})
                continue
            try:
                left, top, right, bottom = bbox_center_region(bbox, width, height, fraction=0.5)
            except ValueError as exc:
                rejected.append({"reason": "invalid_bbox: %s" % exc, "detection": detection})
                continue
            values = []
            for v in range(top, bottom, sample_step):
                for u in range(left, right, sample_step):
                    if 0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]:
                        values.append(float(depth[v, u]))
            valid = [v for v in values if math.isfinite(v) and v > 0.0]
            ratio = len(valid) / float(len(values)) if values else 0.0
            if ratio < float(self.get_parameter("min_valid_ratio").value):
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
            item.update(
                {
                    "depth_m": median,
                    "point_camera": list(point_camera),
                    "point_map": point_map,
                    "depth_samples": len(valid),
                    "depth_valid_ratio": ratio,
                    "depth_mad_m": mad,
                }
            )
            accepted.append(item)

        status = (
            "ready"
            if accepted
            else (rejected[0]["reason"] if rejected else "no_detections")
        )
        self._publish_result(payload, accepted, rejected, status)


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
