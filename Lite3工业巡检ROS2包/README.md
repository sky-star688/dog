# Lite3 双狗协同系统工作空间

两条 DeepRobotics Lite3 机器狗在同一 ROS 2 域下协同：

```
Dog 1 (lite3_01)  建图巡检狗  LiDAR + 深度相机  SLAM 建图 + 航点巡检 + 目标发现
Dog 2 (lite3_02)  拾取作业狗  OpenMV + 深度相机 + MyArm-2  精定位 + 抓取/放置
fleet_manager     协调器      心跳 / 目标分配 / 作业区预约 / 交接握手 / 急停 / 报告
```

串行交接闭环：

```
Dog 1 巡逻 → 发现 block → 暂停 → 上报 fleet_manager
   → fleet_manager 检查 Dog 2 状态 → 预约作业区 → 分配目标
   → Dog 2 导航至接近位 → OpenMV 精定位 → PICK → PLACE → 撤退 → 上报完成
   → fleet_manager 释放作业区 → 通知 Dog 1 → Dog 1 继续巡检
```

## 工作空间结构

```
Lite3工业巡检ROS2包/
├── src/
│   ├── fleet_interfaces/      # 自定义 ROS2 msg/srv/action (CMake)
│   ├── lite3_inspection/      # 共享库 (Python)：领域逻辑、协议、安全链
│   ├── fleet_manager/         # 协调器：心跳、交接、作业区、报告
│   ├── lite3_01_mapping/      # 🗺️ Dog 1 – LiDAR SLAM + 巡检
│   └── lite3_02_picking/      # 🦾 Dog 2 – OpenMV + MyArm-2 拾取
├── scripts/                   # 编译/启动/测试脚本
├── models/                    # YOLO 模型与标签
└── README.md
```

## 快速开始

```bash
# 编译全部
bash scripts/build.sh

# 只编译某条狗（独立调试时）
bash scripts/build.sh dog1
bash scripts/build.sh dog2
bash scripts/build.sh fleet

# 一键启动全系统
bash scripts/run_dual.sh
```

## 独立调试两条狗

### Dog 1（建图巡检）

```bash
bash scripts/run_dog1.sh

# 开始/暂停/停止巡逻
ros2 service call /dog1/start_patrol std_srvs/srv/Trigger "{}"
ros2 service call /dog1/pause_patrol std_srvs/srv/Trigger "{}"
ros2 service call /dog1/stop_patrol std_srvs/srv/Trigger "{}"

# 冻结地图给 Dog 2（关键！）
ros2 service call /dog1/freeze_map std_srvs/srv/Trigger "{}"
ros2 topic echo /shared_map/metadata
```

### Dog 2（拾取作业）

```bash
bash scripts/run_dog2.sh

# 查看地图校验和定位状态（两者都必须为 true）
ros2 topic echo /dog2/map_status
ros2 topic echo /dog2/localization_status

# 大模型识别：即时查询 / 状态
ros2 service call /dog2/vlm_query std_srvs/srv/Trigger "{}"
ros2 topic echo /dog2/vlm_status
ros2 topic echo /dog2/vlm_detections

# 定点重定位 / 机械臂
ros2 service call /dog2/set_initial_pose std_srvs/srv/Trigger "{}"
ros2 service call /dog2/relocalize std_srvs/srv/Trigger "{}"
ros2 service call /dog2/home std_srvs/srv/Trigger "{}"
ros2 service call /dog2/stop std_srvs/srv/Trigger "{}"
```

### 大模型识别接入（可选，默认 mock）

识别准入在 `src/lite3_02_picking/config/picking_params.yaml`：

| 参数 | 作用 |
|------|------|
| `vlm_detector_dog2.backend` | `mock`（离线台架）或 `http`（OpenAI 兼容服务） |
| `vlm_detector_dog2.endpoint/model` | 本地 vLLM/Ollama 或云端 API |
| `vlm_detector_dog2.api_key_env` | 云端密钥的环境变量名 |
| `picking_manager_dog2.target_source` | `openmv` / `vlm` / `any` |
| `picking_manager_dog2.vlm_required` | openmv 模式下是否还要求大模型确认 |

完整契约与后端示例见
[`src/lite3_inspection/docs/large_model_interface.md`](src/lite3_inspection/docs/large_model_interface.md)。
大模型结果只作识别建议，PICK 仍需通过识别准入与 OpenMV/深度/机械臂门控。

### 协调器

```bash
bash scripts/run_fleet.sh
ros2 topic echo /fleet/status
ros2 service call /fleet/generate_report std_srvs/srv/Trigger "{}"
```

## ⚠️ 地图漂移与 Dog 2 初始定位（本系统最关键的坑）

问题：Dog 1 实时 SLAM 建图时地图每帧都在变化。Dog 2 用 AMCL 在“移动的地图”上
定位，必然漂移，表现为「第二条狗找不到自己的初始定位」。

解决：**冻结协议（Freeze Protocol）**

```
1. Dog 1 建图完成 → 冻结
     ros2 service call /dog1/freeze_map std_srvs/srv/Trigger "{}"
     → 返回 "map frozen v1 sha256=..."
     → /shared_map 变为静态快照，/shared_map/metadata 发布 frozen=true + sha256

2. Dog 2 接收冻结地图 → 校验哈希
     对比 metadata 的 sha256 与实际 OccupancyGrid 数据的哈希
     只有完全一致才转发到本地 /map（供 Nav2 + AMCL 使用）
     不一致 → 告警并拒绝使用

3. Dog 2 设置初始位姿 → AMCL 收敛
     config/picking.yaml 的 localization.initial_x/y/yaw 必须是
     Dog 2 在共享地图中的真实测绘位置；错误位姿会导致 AMCL 永不收敛
     dog2_localization 等待协方差收敛后发布 /dog2/localization_status=true

4. 准入条件
     picking_manager 只有在 map_verified=true 且 localized=true 时
     才接受 fleet_manager 的目标分配
```

重定位流程（Dog 2 丢失定位时）：

```bash
ros2 topic echo /dog2/localization_info        # 看 cov_xy，>1.0 即发散
ros2 service call /dog2/relocalize std_srvs/srv/Trigger "{}"   # 全局重定位
ros2 service call /dog2/set_initial_pose std_srvs/srv/Trigger "{}"  # 重设初始位姿
```

| 现象 | 原因 | 解决 |
|------|------|------|
| map_verified 始终 false | 哈希不匹配 | Dog 1 重新 freeze；检查 /shared_map 话题 |
| localization_status 始终 false | 初始位姿不对 | RViz 确认 Dog 2 真实位姿，更新 picking.yaml |
| cov_xy 在 0.5~1.0 徘徊 | 粒子太散 | 减小 initial_cov_xy，确保雷达特征充足 |
| 定位一会就丢 | 地图特征不足 | Dog 1 多建图后再 freeze |
| 导航偏 | /map 冲突 | 确认只有 map_subscriber 发布本地 /map |

## 双狗协同测试流程

| 阶段 | 内容 | Dog 1 | Dog 2 | Fleet |
|------|------|:-----:|:-----:|:-----:|
| A | 各狗独立启动，确认话题/TF/相机 | ✅ | ✅ | ❌ |
| B | Dog 1 建图 → freeze，确认 /shared_map | ✅ | ✅(订阅) | ❌ |
| B2 | **Dog 2 初始位姿设定 → AMCL 收敛** | ❌ | ✅ | ❌ |
| C | 启动协调器，确认心跳 | ✅ | ✅ | ✅ |
| D | Dog 1 单航点巡逻，不自动抓取 | ✅ | ❌ | ✅ |
| E | Dog 2 机械臂台架（空载 dry-run） | ❌ | ✅ | ❌ |
| F | Dog 2 OpenMV 台架（只读→标定→KMS） | ❌ | ✅ | ❌ |
| G | 串行交接 dry-run（有人监护、无负载） | ✅ | ✅ | ✅ |
| H | 完整抓取（A-G 全部通过后） | ✅ | ✅ | ✅ |

## 服务速查

| 服务 | 说明 |
|------|------|
| `/dog1/start_patrol` / `pause_patrol` / `stop_patrol` | Dog 1 巡逻控制 |
| `/dog1/freeze_map` / `/dog1/unfreeze_map` | 冻结/解冻地图 |
| `/dog1/map_snapshot` | 查询冻结地图哈希与元数据 |
| `/dog1/save_map` | 保存 SLAM 地图 |
| `/dog2/set_initial_pose` | 向 AMCL 重发初始位姿 |
| `/dog2/relocalize` | 全局重定位 |
| `/dog2/home` / `/dog2/stop` | 机械臂回中 / 停止 |
| `/fleet/stop_all` / `/fleet/emergency_stop` | 双狗停止 / 急停 |
| `/fleet/resume_all` / `/fleet/return_all` | 双狗恢复 / 返回 |
| `/fleet/generate_report` | 生成编队报告 |

## 故障排查

见 [src/lite3_inspection/docs/OPERATIONS.md](src/lite3_inspection/docs/OPERATIONS.md)。
