# 实现架构

日期：2026-11-11
范围：`Lite3工业巡检ROS2包` 工作空间（双狗协同 + 遗留单狗栈）

## 1. 双狗协同架构

```
                        ┌──────────────────────────┐
                        │      fleet_manager        │
                        │   (地面站/协调器)          │
                        │  心跳 / 目标分配 / 作业区   │
                        │  交接握手 / 急停 / 报告     │
                        └──────┬──────────┬─────────┘
                               │          │
               /fleet/heartbeat│          │/fleet/heartbeat
               /fleet/target   │          │/lite3_02/target_assignment
                               │          │/fleet/handoff_status
                      ┌────────▼──┐  ┌───▼──────────────┐
                      │ lite3_01  │  │    lite3_02      │
                      │ 建图巡检狗 │  │    拾取作业狗      │
                      │ LiDAR+深度 │  │ OpenMV+深度+MyArm │
                      └───────────┘  └───────────────────┘
                               │          │
                      /shared_map（冻结快照 + SHA256）
```

串行交接：巡检狗发现目标 → 暂停/上报 → 协调器预约作业区并分配 →
作业狗接近/精定位/抓取/放置/撤退 → 协调器释放作业区并通知巡检狗继续。

## 2. 部署分层

```
lite3_inspection（共享库 + 遗留栈）
  geometry / robots / safety / perception / mission / arm / openmv / events
  cmd_vel_mux / cmd_vel_guard / localization_monitor / arm_serial_bridge
  vision_detector / depth_projector / event_recorder / inspection_manager

lite3_01_mapping（Dog 1）
  map_bridge（冻结+哈希）/ mapping_manager / dog1_vision_detector
  dog1_depth_projector / patrol_manager

lite3_02_picking（Dog 2）
  map_subscriber（哈希校验）/ dog2_localization（AMCL 初始位姿+收敛）
  openmv_positioner / dog2_arm_bridge
  vlm_detector（大模型识别，可插拔：http / mock）
  picking_manager（识别准入 → PICK/PLACE）

fleet_manager（协调器）
  fleet_state / handoff / workspace / fleet_safety / fleet_manager
```

### 2.1 大模型识别接口

```
相机帧 → vlm_detector_dog2 ──HTTP(OpenAI 兼容)──► 大模型服务
        │  失败时显式 detector_status，空检测
        │  /dog2/vlm_detections (JSON，与 YOLO 检测同构)
        ▼
picking_manager 识别准入（target_source=openmv|vlm|any，vlm_required）
        ▼
OpenMV/深度门控 → PICK → PLACE
```

大模型结果只作建议，不能直接控制机械臂；HTTP 在独立线程执行，急停不被阻塞。
详见 `docs/large_model_interface.md`。

## 3. 状态所有权

| 状态 | 所有者 | 消费者 |
| --- | --- | --- |
| `motion_enable`（每狗） | patrol_manager / picking_manager | mux、guard |
| `estop`（每狗） | 外部急停 | mux、manager、fleet |
| `map frozen + sha256` | Dog 1 map_bridge | Dog 2 map_subscriber、picking_manager |
| `localization`（Dog 2） | dog2_localization（AMCL） | picking_manager |
| 交接状态 | fleet_manager + HandoffState | 两狗、报告 |
| 作业区预约 | fleet_manager workspace | 两狗 |

## 4. 失败语义

- 无效速度整条拒绝，不转发部分轴；超时输出零。
- 定位不 ready 时 manager 关闭运动使能并暂停任务。
- 地图未冻结/哈希不匹配：Dog 2 不转发本地 `/map`，拒绝目标。
- AMCL 未收敛：picking_manager 拒绝目标分配。
- 机械臂 `$DGS` 无完成 ACK：按实测忙窗推进，绝不把写串口当完成。
- 交接超时 600 s：fleet_manager 判失败、释放作业区、巡检狗跳过目标继续。

## 5. 已知缺口

- 多机 namespace 未在所有节点强制隔离（话题以 `/dog1`、`/dog2` 前缀区分）。
- 无 Gazebo/Isaac 仿真回归。
- OpenMV 现场自动标定未接入 ROS 任务链（主机工具已就绪）。
- 无动作组完成反馈，忙窗需现场实测。
- 并行巡检模式未实现（当前仅串行交接）。
