# lite3_inspection

Lite3 巡检共享库与遗留单狗栈。

## 职责

- 领域层（无 ROS 依赖）：`geometry`、`robots`、`safety`、`perception`、
  `mission`、`arm`、`openmv`、`events`、`fleet_client`
- ROS 节点：`cmd_vel_mux`、`cmd_vel_guard`、`localization_monitor`、
  `arm_serial_bridge`、`vision_detector`、`depth_projector`、
  `event_recorder`、`inspection_manager`
- 大模型接口：`vlm`（HTTP/OpenAI 兼容 + mock，结果 JSON 契约）

双狗包（`lite3_01_mapping`、`lite3_02_picking`）复用本包的领域层与安全节点。

## 测试

```bash
PYTHONPATH=$PWD python3 -m pytest test/ -v
```

## 文档

- `docs/ARCHITECTURE.md` 架构与状态所有权
- `docs/interfaces.md` 话题/服务/参数契约
- `docs/safety.md` 安全边界与上线门槛
- `docs/arm_integration.md` MyArm-2 集成
- `docs/openmv_vision.md` OpenMV 链路与标定
- `docs/build_and_deploy.md` 构建与部署
- `docs/testing_and_acceptance.md` 验收阶段
- `docs/OPERATIONS.md` 双狗操作与故障排查
- `docs/large_model_interface.md` 大模型识别接口与后端配置
