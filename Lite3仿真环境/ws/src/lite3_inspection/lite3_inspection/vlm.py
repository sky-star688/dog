"""Large-model (VLM/LLM) perception interface — ROS-free domain logic.

Purpose: let a vision-language model recognise the target and return results
in the same detection schema the rest of the stack already understands, so the
picking dog can gate on model output and then grasp.

This module deliberately does NOT talk to hardware or ROS.  It provides:
  - VlmConfig:          backend/model/prompt/timeout/limits
  - build_chat_payload: OpenAI-compatible request body with base64 image
  - extract_json:       tolerant JSON extraction from model prose/code fences
  - parse_detections:   normalise several response schemas into detections
  - detections_message: stable JSON contract published to ROS
  - MockBackend:        offline, explicit, never pretends to be a real model

Safety: a VLM result is advisory.  It never writes to the arm directly; the
picking manager must still pass localisation, depth/OpenMV and arm readiness
gates before PICK/PLACE.
"""

from __future__ import annotations

import base64
import json
import math
import os
import re
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence


VLM_BACKENDS = ("mock", "http")
MAX_IMAGE_BYTES = 8 * 1024 * 1024
MAX_TEXT_CHARS = 4000
DEFAULT_PROMPT = (
    "你是机器狗视觉识别模块。找出画面中需要抓取的目标。"
    "只输出 JSON，不要输出解释，格式："
    '{"detections":[{"class_name":"block","confidence":0.9,'
    '"bbox":[x1,y1,x2,y2],"grasp_point":[u,v]}]}。'
    "bbox 使用图像像素坐标；没有目标时输出 {\"detections\":[]}。"
)


class VlmError(Exception):
    """Raised when a VLM request or response cannot be used safely."""

    def __init__(self, reason: str, status: str = "model_error") -> None:
        super().__init__(reason)
        self.reason = reason
        self.status = status


@dataclass(frozen=True)
class VlmConfig:
    backend: str = "mock"
    endpoint: str = ""
    model: str = ""
    api_key_env: str = "VLM_API_KEY"
    prompt: str = DEFAULT_PROMPT
    timeout_sec: float = 8.0
    retries: int = 1
    min_confidence: float = 0.50
    allowed_classes: Sequence[str] = ("block",)
    max_tokens: int = 512
    use_json_mode: bool = False

    def validate(self) -> List[str]:
        errors = []
        if self.backend not in VLM_BACKENDS:
            errors.append("backend must be one of %s" % (VLM_BACKENDS,))
        if self.backend == "http" and not self.endpoint:
            errors.append("http backend requires endpoint")
        if self.backend == "http" and not self.model:
            errors.append("http backend requires model")
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            errors.append("timeout_sec must be positive")
        if not isinstance(self.retries, int) or self.retries < 0:
            errors.append("retries must be a non-negative integer")
        if not 0.0 <= self.min_confidence <= 1.0:
            errors.append("min_confidence must be in [0, 1]")
        if not self.prompt or not str(self.prompt).strip():
            errors.append("prompt must not be empty")
        if not self.allowed_classes:
            errors.append("allowed_classes must not be empty")
        return errors

    def api_key(self) -> str:
        return os.environ.get(self.api_key_env, "").strip()


@dataclass
class VlmDetection:
    class_name: str
    confidence: float
    bbox: List[float]
    grasp_point: Optional[List[float]] = None
    label: str = ""

    @property
    def center(self) -> List[float]:
        return [
            (self.bbox[0] + self.bbox[2]) * 0.5,
            (self.bbox[1] + self.bbox[3]) * 0.5,
        ]

    def to_dict(self) -> Dict[str, Any]:
        payload = {
            "class_name": self.class_name,
            "label": self.label or self.class_name,
            "confidence": round(float(self.confidence), 4),
            "bbox": [float(v) for v in self.bbox],
            "center": self.center,
        }
        if self.grasp_point is not None:
            payload["grasp_point"] = [float(v) for v in self.grasp_point]
        return payload


def _encode_image(image_bytes: bytes) -> str:
    if not isinstance(image_bytes, (bytes, bytearray)):
        raise VlmError("image must be bytes", "no_frame")
    if not image_bytes:
        raise VlmError("image is empty", "no_frame")
    if len(image_bytes) > MAX_IMAGE_BYTES:
        raise VlmError("image exceeds %d bytes" % MAX_IMAGE_BYTES, "no_frame")
    encoded = base64.b64encode(bytes(image_bytes)).decode("ascii")
    return "data:image/jpeg;base64," + encoded


def build_chat_payload(
    config: VlmConfig, image_bytes: bytes, prompt: Optional[str] = None
) -> Dict[str, Any]:
    """Build an OpenAI-compatible chat-completions request body."""
    errors = config.validate()
    if errors:
        raise VlmError("; ".join(errors), "config_error")
    body = {
        "model": config.model,
        "max_tokens": int(config.max_tokens),
        "temperature": 0.0,
        "messages": [
            {
                "role": "system",
                "content": (
                    "You are a robot vision detector. Answer with JSON only; "
                    "never invent objects that are not visible."
                ),
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt or config.prompt},
                    {
                        "type": "image_url",
                        "image_url": {"url": _encode_image(image_bytes)},
                    },
                ],
            },
        ],
    }
    if config.use_json_mode:
        body["response_format"] = {"type": "json_object"}
    return body


def _strip_code_fences(text: str) -> str:
    cleaned = text.strip()
    fence = re.search(r"```(?:json|JSON)?\s*(.*?)```", cleaned, re.DOTALL)
    if fence:
        cleaned = fence.group(1).strip()
    return cleaned


def extract_json(text: str) -> Dict[str, Any]:
    """Extract the first balanced JSON object from model output."""
    if not isinstance(text, str) or not text.strip():
        raise VlmError("model output is empty", "parse_error")
    if len(text) > MAX_TEXT_CHARS:
        text = text[:MAX_TEXT_CHARS]
    cleaned = _strip_code_fences(text)

    start = cleaned.find("{")
    if start < 0:
        raise VlmError("model output contains no JSON object", "parse_error")
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(cleaned)):
        char = cleaned[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                candidate = cleaned[start:index + 1]
                try:
                    payload = json.loads(candidate)
                except Exception as exc:
                    raise VlmError("invalid JSON: %s" % exc, "parse_error")
                if not isinstance(payload, dict):
                    raise VlmError("JSON root must be an object", "parse_error")
                return payload
    raise VlmError("unbalanced JSON object", "parse_error")


def _coerce_bbox(value: Any) -> List[float]:
    if isinstance(value, dict):
        try:
            value = [value["x1"], value["y1"], value["x2"], value["y2"]]
        except KeyError:
            raise VlmError("bbox object must contain x1,y1,x2,y2", "parse_error")
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise VlmError("bbox must be four numbers", "parse_error")
    try:
        numbers = [float(v) for v in value]
    except (TypeError, ValueError):
        raise VlmError("bbox values must be numbers", "parse_error")
    if not all(math.isfinite(v) for v in numbers):
        raise VlmError("bbox values must be finite", "parse_error")
    x1, y1, x2, y2 = numbers
    if x2 <= x1 or y2 <= y1:
        raise VlmError("bbox must have positive area", "parse_error")
    return numbers


def _coerce_point(value: Any) -> Optional[List[float]]:
    if value is None:
        return None
    if isinstance(value, dict):
        try:
            value = [value["u"], value["v"]]
        except KeyError:
            try:
                value = [value["x"], value["y"]]
            except KeyError:
                raise VlmError("grasp_point must contain u,v", "parse_error")
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise VlmError("grasp_point must be two numbers", "parse_error")
    try:
        numbers = [float(v) for v in value]
    except (TypeError, ValueError):
        raise VlmError("grasp_point values must be numbers", "parse_error")
    if not all(math.isfinite(v) for v in numbers):
        raise VlmError("grasp_point values must be finite", "parse_error")
    return numbers


def parse_detections(
    payload: Dict[str, Any], config: VlmConfig
) -> List[VlmDetection]:
    """Normalise model output into validated detections.

    Accepts ``detections``/``objects``/``results`` lists, or a single object
    with ``class_name``/``label``.  Unknown classes and low confidence are
    dropped; malformed entries raise VlmError (fail closed, never guess).
    """
    if not isinstance(payload, dict):
        raise VlmError("model payload must be an object", "parse_error")

    rows: Any = None
    for key in ("detections", "objects", "results"):
        if key in payload:
            rows = payload[key]
            break
    if rows is None:
        if "class_name" in payload or "label" in payload:
            rows = [payload]
        elif payload.get("detections") is None and not payload:
            rows = []
        else:
            rows = []
    if not isinstance(rows, list):
        raise VlmError("detections must be a list", "parse_error")

    allowed = {str(name) for name in config.allowed_classes}
    detections: List[VlmDetection] = []
    for row in rows:
        if not isinstance(row, dict):
            raise VlmError("each detection must be an object", "parse_error")
        class_name = str(row.get("class_name", row.get("label", ""))).strip()
        if not class_name:
            raise VlmError("detection has no class_name", "parse_error")
        try:
            confidence = float(row.get("confidence", row.get("score", 0.0)))
        except (TypeError, ValueError):
            raise VlmError("confidence must be a number", "parse_error")
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise VlmError("confidence must be in [0, 1]", "parse_error")
        bbox = _coerce_bbox(row.get("bbox", row.get("box")))
        grasp_point = _coerce_point(row.get("grasp_point"))
        if class_name not in allowed:
            continue
        if confidence < config.min_confidence:
            continue
        detections.append(
            VlmDetection(
                class_name=class_name,
                confidence=confidence,
                bbox=bbox,
                grasp_point=grasp_point,
                label=str(row.get("label", "")),
            )
        )
    return detections


def parse_response(
    response: Dict[str, Any], config: VlmConfig
) -> tuple:
    """Extract detections and raw text from an OpenAI-style or plain response."""
    if not isinstance(response, dict):
        raise VlmError("response must be an object", "parse_error")

    text = ""
    choices = response.get("choices")
    if isinstance(choices, list) and choices:
        first = choices[0] if isinstance(choices[0], dict) else {}
        message = first.get("message") if isinstance(first.get("message"), dict) else {}
        text = str(message.get("content", "") or first.get("text", "") or "")
    elif isinstance(response.get("response"), str):
        text = response["response"]
    elif "detections" in response or "objects" in response:
        payload = response
    else:
        raise VlmError("response has no message content", "parse_error")

    if text:
        payload = extract_json(text)
    detections = parse_detections(payload, config)
    return detections, text


def detections_message(
    detections: Sequence[VlmDetection],
    config: VlmConfig,
    status: str = "ready",
    frame_id: str = "",
    latency_sec: Optional[float] = None,
    raw_text: str = "",
    reason: str = "",
) -> str:
    """Stable JSON contract published on the VLM detection topic."""
    payload: Dict[str, Any] = {
        "detector_status": status,
        "backend": "vlm_%s" % config.backend,
        "model": config.model,
        "source": "vlm",
        "frame_id": frame_id,
        "detections": [detection.to_dict() for detection in detections],
        "stamp": {"sec": int(time.time()), "nanosec": 0},
    }
    if latency_sec is not None:
        payload["latency_sec"] = round(float(latency_sec), 3)
    if raw_text:
        payload["raw_text"] = raw_text[:500]
    if reason:
        payload["reason"] = reason
    return json.dumps(payload, ensure_ascii=False)


class MockBackend:
    """Offline backend for bench tests.  Always labelled backend=vlm_mock."""

    def __init__(self, detections_json: str = "") -> None:
        self._detections_json = detections_json or ""

    def query(self, image_bytes: bytes, config: VlmConfig):
        started = time.time()
        if not self._detections_json.strip():
            return [], "", time.time() - started
        payload = extract_json(self._detections_json)
        detections = parse_detections(payload, config)
        return detections, self._detections_json, time.time() - started


def http_query(
    image_bytes: bytes,
    config: VlmConfig,
    prompt: Optional[str] = None,
) -> tuple:
    """POST one chat-completions request using urllib (no extra deps)."""
    import urllib.error
    import urllib.request

    body = build_chat_payload(config, image_bytes, prompt)
    data = json.dumps(body).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    api_key = config.api_key()
    if api_key:
        headers["Authorization"] = "Bearer %s" % api_key

    started = time.time()
    last_error = None
    for attempt in range(int(config.retries) + 1):
        try:
            request = urllib.request.Request(
                config.endpoint, data=data, headers=headers, method="POST"
            )
            with urllib.request.urlopen(request, timeout=config.timeout_sec) as response:
                raw = response.read().decode("utf-8", errors="replace")
            try:
                response_json = json.loads(raw)
            except Exception as exc:
                raise VlmError("backend returned non-JSON: %s" % exc, "parse_error")
            detections, text = parse_response(response_json, config)
            return detections, text, time.time() - started
        except VlmError:
            raise
        except Exception as exc:
            last_error = exc
            if attempt < int(config.retries):
                time.sleep(min(0.5 * (attempt + 1), 2.0))
                continue
            raise VlmError("backend request failed: %s" % exc, "backend_unavailable")
    raise VlmError("backend request failed: %s" % last_error, "backend_unavailable")
