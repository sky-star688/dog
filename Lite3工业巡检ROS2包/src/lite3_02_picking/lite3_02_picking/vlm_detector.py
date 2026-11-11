"""Dog 2 large-model (VLM) detector node.

Turns camera frames into large-model detections and publishes them in the same
JSON schema as the YOLO detector, so the picking manager can gate on them and
then grasp.

Backends:
  - ``http``: any OpenAI-compatible chat-completions server (vLLM, Ollama,
    LM Studio, Qwen-VL API, ...).  Image is sent as base64 data URL.
  - ``mock``: offline bench backend; requires ``mock_detections`` JSON to emit
    anything.  Always labelled ``backend="vlm_mock"`` so it can never be
    mistaken for a real model result.

Safety:
  - HTTP work runs in a worker thread so estop/arm topics keep spinning.
  - Failures publish an explicit ``detector_status`` and empty detections.
  - A VLM result is advisory: it never writes to the arm directly.
"""

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from lite3_inspection.vlm import (
    DEFAULT_PROMPT,
    MockBackend,
    VlmConfig,
    VlmError,
    detections_message,
    http_query,
)

try:
    import cv2
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - optional on a dev machine
    cv2 = None
    CvBridge = None


class VlmDetector(Node):
    def __init__(self):
        super().__init__("vlm_detector_dog2")

        self.declare_parameter("backend", "mock")
        self.declare_parameter("endpoint", "")
        self.declare_parameter("model", "")
        self.declare_parameter("api_key_env", "VLM_API_KEY")
        self.declare_parameter("prompt", DEFAULT_PROMPT)
        self.declare_parameter("timeout_sec", 8.0)
        self.declare_parameter("retries", 1)
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("allowed_classes", ["block"])
        self.declare_parameter("max_tokens", 512)
        self.declare_parameter("use_json_mode", False)
        self.declare_parameter("image_topic", "/dog2/camera/color/image_raw")
        self.declare_parameter("output_topic", "/dog2/vlm_detections")
        self.declare_parameter("status_topic", "/dog2/vlm_status")
        self.declare_parameter("process_every_n_frames", 15)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("mock_detections", "")
        self.declare_parameter("enabled", True)

        self.config = VlmConfig(
            backend=str(self.get_parameter("backend").value),
            endpoint=str(self.get_parameter("endpoint").value),
            model=str(self.get_parameter("model").value),
            api_key_env=str(self.get_parameter("api_key_env").value),
            prompt=str(self.get_parameter("prompt").value),
            timeout_sec=float(self.get_parameter("timeout_sec").value),
            retries=int(self.get_parameter("retries").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            allowed_classes=tuple(
                str(v) for v in self.get_parameter("allowed_classes").value
            ),
            max_tokens=int(self.get_parameter("max_tokens").value),
            use_json_mode=bool(self.get_parameter("use_json_mode").value),
        )
        errors = self.config.validate()
        if errors:
            self.get_logger().error("Invalid VLM config: %s" % "; ".join(errors))

        self._enabled = bool(self.get_parameter("enabled").value)
        self._every = max(1, int(self.get_parameter("process_every_n_frames").value))
        self._quality = min(95, max(30, int(self.get_parameter("jpeg_quality").value)))
        self._mock = MockBackend(str(self.get_parameter("mock_detections").value))

        self._frame_count = 0
        self._latest_frame: Optional[bytes] = None
        self._latest_header = None
        self._frame_lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()

        self._last_status = "starting"
        self._last_latency = 0.0
        self._last_count = 0
        self._last_error = ""
        self._last_result_json = "{}"

        self.bridge = CvBridge() if CvBridge is not None else None

        self.output_pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, "/dog2/vlm_query", self._on_query)
        self.create_timer(2.0, self._publish_status)

        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "VLM detector ready: backend=%s model=%s endpoint=%s"
            % (self.config.backend, self.config.model or "-", self.config.endpoint or "-")
        )
        if self.config.backend == "http" and not self.config.api_key():
            self.get_logger().warn(
                "VLM API key env '%s' is empty; set it or use a local server"
                % self.config.api_key_env
            )

    # ---- Input ----

    def _on_image(self, message: Image) -> None:
        self._frame_count += 1
        if self._frame_count % self._every != 0:
            return
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, buffer = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality]
            )
            if not ok:
                raise RuntimeError("jpeg encode failed")
        except Exception as exc:
            self._set_status("no_frame", reason="image decode failed: %s" % exc)
            return
        with self._frame_lock:
            self._latest_frame = buffer.tobytes()
            self._latest_header = message.header
        self._wake.set()

    # ---- Worker ----

    def _run_worker(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            if not self._enabled:
                continue
            with self._frame_lock:
                frame = self._latest_frame
                header = self._latest_header
            if frame is None:
                self._set_status("no_frame", reason="no camera frame yet")
                continue
            self._query_once(frame, header)

    def _query_once(self, frame: bytes, header) -> None:
        frame_id = header.frame_id if header is not None else ""
        try:
            if self.config.backend == "mock":
                detections, text, latency = self._mock.query(frame, self.config)
                status = "ready" if detections else "no_detections"
            else:
                detections, text, latency = http_query(frame, self.config)
                status = "ready"
        except VlmError as exc:
            self._set_status(exc.status, reason=exc.reason)
            self.output_pub.publish(
                String(
                    data=detections_message(
                        [], self.config, status=exc.status,
                        frame_id=frame_id, reason=exc.reason,
                    )
                )
            )
            return
        except Exception as exc:
            self._set_status("model_error", reason=str(exc))
            return

        message = detections_message(
            detections,
            self.config,
            status=status,
            frame_id=frame_id,
            latency_sec=latency,
            raw_text=text,
        )
        self._last_result_json = message
        self._last_latency = latency
        self._last_count = len(detections)
        self._set_status(status)
        self.output_pub.publish(String(data=message))
        if detections:
            self.get_logger().info(
                "VLM detections: %d (best %s %.2f, %.3f s)"
                % (
                    len(detections),
                    detections[0].class_name,
                    detections[0].confidence,
                    latency,
                )
            )

    # ---- Service ----

    def _on_query(self, request, response):
        del request
        with self._frame_lock:
            frame = self._latest_frame
            header = self._latest_header
        if frame is None:
            response.success = False
            response.message = "no camera frame available"
            return response
        self._query_once(frame, header)
        response.success = self._last_result_json != "{}"
        response.message = self._last_result_json
        return response

    # ---- Status ----

    def _set_status(self, status: str, reason: str = "") -> None:
        if status != self._last_status or reason != self._last_error:
            self.get_logger().info("VLM status: %s %s" % (status, reason))
        self._last_status = status
        self._last_error = reason

    def _publish_status(self) -> None:
        payload = {
            "status": self._last_status,
            "backend": self.config.backend,
            "model": self.config.model,
            "endpoint": self.config.endpoint,
            "latency_sec": round(self._last_latency, 3),
            "last_detection_count": self._last_count,
            "last_error": self._last_error,
            "enabled": self._enabled,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def destroy_node(self):
        self._stop.set()
        self._wake.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VlmDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
