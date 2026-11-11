# 大模型（VLM）识别接口

拾取狗（lite3_02）可以接任意视觉语言大模型完成识别，结果回传机器人并用于抓取准入。
接口是**可插拔**的：本地 vLLM / Ollama / LM Studio、云端 OpenAI 兼容 API，或离线 mock。

## 1. 数据流

```
Dog 2 相机 (/dog2/camera/color/image_raw)
        │
        ▼
vlm_detector_dog2  ──HTTP/OpenAI 兼容──►  大模型服务
        │                                  (vLLM / Ollama / 云端)
        │  /dog2/vlm_detections (JSON)
        ▼
picking_manager ──► 识别准入 (target_source / vlm_required)
        │
        ▼
OpenMV 精定位 / 深度 ──► PICK → PLACE → 撤退
```

大模型结果只是**建议**：它不能直接写机械臂，必须再通过定位、深度/OpenMV
与机械臂就绪门控，才会发出 `PICK`。

## 2. 输出契约（`/dog2/vlm_detections`）

```json
{
  "detector_status": "ready",
  "backend": "vlm_http",
  "model": "qwen2.5-vl-7b",
  "source": "vlm",
  "frame_id": "camera_color_optical_frame",
  "stamp": {"sec": 0, "nanosec": 0},
  "latency_sec": 1.23,
  "detections": [
    {
      "class_name": "block",
      "label": "block",
      "confidence": 0.92,
      "bbox": [120, 80, 200, 160],
      "center": [160.0, 120.0],
      "grasp_point": [162.0, 118.0]
    }
  ],
  "raw_text": "..."
}
```

`detector_status` 取值：`ready`、`no_detections`、`no_frame`、
`backend_unavailable`、`parse_error`、`model_error`、`config_error`。
失败时 `detections` 为空，绝不编造目标。

字段与 `vision_detector` 的 2D 检测一致，因此可直接接 `depth_projector`
或复用后端处理。

## 3. 配置（摘录 `picking_params.yaml`）

```yaml
vlm_detector_dog2:
  ros__parameters:
    enabled: true
    backend: http                                  # mock | http
    endpoint: http://127.0.0.1:8000/v1/chat/completions
    model: qwen2.5-vl-7b
    api_key_env: VLM_API_KEY                       # 本地服务可留空
    timeout_sec: 8.0
    retries: 1
    min_confidence: 0.50
    allowed_classes: [block]
    process_every_n_frames: 15
    use_json_mode: false                           # 服务端支持才开

picking_manager_dog2:
  ros__parameters:
    target_source: openmv     # openmv | vlm | any
    vlm_required: false       # openmv 模式：是否还要求大模型确认
    require_openmv: true      # vlm 模式：是否仍要求 OpenMV 精定位
    vlm_min_confidence: 0.50
    vlm_stable_frames: 5
    vlm_stale_sec: 3.0
```

识别准入组合：

| target_source | 发 PICK 的条件 |
| --- | --- |
| `openmv`（默认） | OpenMV 锁定 + （`vlm_required` 时大模型连续确认） |
| `vlm` | 大模型连续确认 + （`require_openmv` 时 OpenMV 锁定） |
| `any` | 大模型确认 **或** OpenMV 锁定 |

## 4. 服务

```bash
# 立即对最新帧做一次识别（返回 detections JSON）
ros2 service call /dog2/vlm_query std_srvs/srv/Trigger "{}"

# 状态：backend / model / latency / last_error
ros2 topic echo /dog2/vlm_status
```

## 5. 后端示例

### 本地 vLLM（OpenAI 兼容）

```bash
python -m vllm.entrypoints.openai.api_server \
  --model Qwen/Qwen2.5-VL-7B-Instruct --port 8000
```

`backend: http`，`endpoint: http://127.0.0.1:8000/v1/chat/completions`，
`model: Qwen/Qwen2.5-VL-7B-Instruct`，无需 API key。

### Ollama（OpenAI 兼容层）

```bash
ollama pull llama3.2-vision
# endpoint: http://127.0.0.1:11434/v1/chat/completions
# model:    llama3.2-vision
```

### 云端 API

设置环境变量后启动：

```bash
export VLM_API_KEY=sk-...
ros2 launch lite3_02_picking picking_dog.launch.py
```

`endpoint` / `model` 按服务商填写；图片以 base64 data URL 发送。

### 离线 mock（台架/回归）

```yaml
vlm_detector_dog2:
  ros__parameters:
    backend: mock
    mock_detections: '{"detections":[{"class_name":"block","confidence":0.9,"bbox":[140,100,180,140]}]}'
```

mock 结果始终标记 `backend: "vlm_mock"`，不会被误认为真实模型输出。

## 6. 安全约束

- 大模型请求在独立线程执行，急停/机械臂话题不会被 HTTP 阻塞。
- 不支持的类别、低置信度、非法 bbox 一律丢弃；解析失败按失败上报。
- 识别结果必须通过 `picking_manager` 门控；VLM 不直接控制机械臂。
- 事件日志记录 `vlm_verified` 阶段，便于追溯“哪一帧、哪个模型、什么置信度”。

## 6.1 时延与超时

VLM 单次推理可能数秒。若使用较慢的云端模型，注意两项要匹配：

- `vlm_detector_dog2.process_every_n_frames`：控制查询频率；
- `picking_manager_dog2.positioning_timeout_sec`（默认 30 s）：
  必须大于 `vlm_stable_frames × 单次时延 + 余量`，
  否则出现 `vlm_positioning_timeout`。

保守起步：`vlm_stable_frames: 3`、`process_every_n_frames: 10`、
`positioning_timeout_sec: 60`，实测后再收紧。

## 7. 强类型接入（可选）

`fleet_interfaces` 提供 `msg/VlmDetection.msg` 与 `srv/VlmQuery.srv`。
默认链路使用 JSON 字符串（与检测链一致），需要强类型时自行桥接。
