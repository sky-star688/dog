"""Tests for the large-model (VLM) interface domain logic."""

import json

import pytest

from lite3_inspection.vlm import (
    DEFAULT_PROMPT,
    MockBackend,
    VlmConfig,
    VlmError,
    build_chat_payload,
    detections_message,
    extract_json,
    parse_detections,
    parse_response,
)


def _config(**kwargs):
    values = dict(
        backend="mock",
        endpoint="",
        model="test-vlm",
        prompt=DEFAULT_PROMPT,
        min_confidence=0.5,
        allowed_classes=("block",),
    )
    values.update(kwargs)
    return VlmConfig(**values)


def test_config_validation():
    assert _config().validate() == []
    errors = _config(backend="http").validate()
    assert errors  # endpoint/model missing
    assert _config(backend="ftp").validate()
    assert _config(timeout_sec=0.0).validate()


def test_build_chat_payload():
    payload = build_chat_payload(_config(), b"\xff\xd8fakejpeg")
    assert payload["model"] == "test-vlm"
    assert payload["temperature"] == 0.0
    content = payload["messages"][1]["content"]
    assert content[0]["type"] == "text"
    assert content[1]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "response_format" not in payload


def test_build_payload_json_mode_and_errors():
    assert build_chat_payload(_config(use_json_mode=True), b"x")["response_format"] == {
        "type": "json_object"
    }
    with pytest.raises(VlmError):
        build_chat_payload(_config(), b"")


def test_extract_json_plain_and_fenced():
    assert extract_json('{"detections": []}') == {"detections": []}
    fenced = "说明文字\n```json\n{\"detections\": []}\n```\n结束"
    assert extract_json(fenced) == {"detections": []}
    prose = '这是结果：{"detections":[{"class_name":"block"}]} 谢谢'
    assert extract_json(prose)["detections"][0]["class_name"] == "block"


def test_extract_json_failures():
    for text in ("", "no json here", '{"detections": [}', '{"a": 1'):
        with pytest.raises(VlmError):
            extract_json(text)


def test_parse_detections_valid_and_filtered():
    payload = {
        "detections": [
            {"class_name": "block", "confidence": 0.9, "bbox": [10, 10, 50, 60]},
            {"class_name": "block", "confidence": 0.2, "bbox": [10, 10, 50, 60]},
            {"class_name": "anomaly", "confidence": 0.9, "bbox": [10, 10, 50, 60]},
        ]
    }
    detections = parse_detections(payload, _config())
    assert len(detections) == 1
    assert detections[0].center == [30.0, 35.0]


def test_parse_detections_grasp_point_and_bbox_dict():
    payload = {
        "objects": [
            {
                "label": "block",
                "score": 0.8,
                "bbox": {"x1": 0, "y1": 0, "x2": 20, "y2": 10},
                "grasp_point": {"u": 10, "v": 5},
            }
        ]
    }
    detection = parse_detections(payload, _config())[0]
    assert detection.grasp_point == [10.0, 5.0]


def test_parse_detections_rejects_malformed():
    for payload in (
        {"detections": [{"class_name": "block", "confidence": 0.9, "bbox": [1, 2, 1, 4]}]},
        {"detections": [{"class_name": "block", "confidence": 1.5, "bbox": [0, 0, 1, 1]}]},
        {"detections": [{"class_name": "", "confidence": 0.9, "bbox": [0, 0, 1, 1]}]},
        {"detections": "not-a-list"},
    ):
        with pytest.raises(VlmError):
            parse_detections(payload, _config())


def test_parse_response_openai_style():
    response = {
        "choices": [
            {
                "message": {
                    "content": '{"detections":[{"class_name":"block","confidence":0.88,"bbox":[5,5,25,25]}]}'
                }
            }
        ]
    }
    detections, text = parse_response(response, _config())
    assert len(detections) == 1
    assert "block" in text


def test_parse_response_plain_and_errors():
    detections, _ = parse_response({"detections": []}, _config())
    assert detections == []
    with pytest.raises(VlmError):
        parse_response({"choices": []}, _config())
    with pytest.raises(VlmError):
        parse_response([], _config())


def test_detections_message_contract():
    detections = parse_detections(
        {"detections": [{"class_name": "block", "confidence": 0.9, "bbox": [0, 0, 10, 10]}]},
        _config(),
    )
    message = json.loads(detections_message(detections, _config(), latency_sec=0.5))
    assert message["detector_status"] == "ready"
    assert message["backend"] == "vlm_mock"
    assert message["source"] == "vlm"
    assert message["detections"][0]["class_name"] == "block"
    assert message["latency_sec"] == 0.5


def test_detections_message_failure_is_explicit():
    message = json.loads(
        detections_message([], _config(), status="backend_unavailable", reason="timeout")
    )
    assert message["detections"] == []
    assert message["detector_status"] == "backend_unavailable"
    assert message["reason"] == "timeout"


def test_mock_backend_empty_and_configured():
    empty = MockBackend("")
    assert empty.query(b"x", _config()) == ([], "", empty.query(b"x", _config())[2])

    mock = MockBackend(
        '{"detections":[{"class_name":"block","confidence":0.9,"bbox":[1,1,9,9]}]}'
    )
    detections, text, latency = mock.query(b"x", _config())
    assert len(detections) == 1
    assert latency >= 0.0
    assert "detections" in text
