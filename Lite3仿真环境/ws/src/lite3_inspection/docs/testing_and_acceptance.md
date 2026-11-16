# 测试与验收

## 离线测试（无需 ROS）

```bash
cd Lite3工业巡检ROS2包/src/lite3_inspection
PYTHONPATH=$PWD python3 -m pytest test/ -v
```

覆盖：安全限幅/仲裁、几何变换、感知投影、任务状态机、机械臂帧校验、
OpenMV 协议与标定、事件存储、机器人身份。

双狗包领域测试：

```bash
cd src/fleet_manager && PYTHONPATH=.:../lite3_inspection python3 -m pytest test/ -v
cd src/lite3_01_mapping && PYTHONPATH=.:../lite3_inspection python3 -m pytest test/ -v
cd src/lite3_02_picking && PYTHONPATH=.:../lite3_inspection python3 -m pytest test/ -v
```

## 实机验收阶段

| 阶段 | 内容 | 通过条件 |
| --- | --- | --- |
| A | 静态启动 | 话题/TF/相机/深度正常，`/cmd_vel` 单发布者 |
| B | Dog 1 建图+冻结 | metadata 有 frozen=true 与 sha256 |
| B2 | Dog 2 定位 | `map_verified=true` 且 `localization_status=true` |
| C | 协调器心跳 | `/fleet/status` 两狗 online |
| D | 单航点巡逻 | 到点/暂停/恢复/停止正常 |
| E | 机械臂台架 | 空载 dry-run，STOP 优先 |
| F | OpenMV 台架 | 只读→标定→单次 KMS |
| G | 交接 dry-run | 上报→分配→接近→精定位→抓取→放置→撤退 |
| H | 完整抓取 | A-G 全通过且有人监护 |

## 判定原则

- 任何“未验证即通过”都记为失败。
- 拒绝原因必须可观测（`/dog2/map_status`、`/dog2/localization_info`、事件日志）。
- 失败后回滚到上一验证配置，不允许跳过阶段直接抓取。
