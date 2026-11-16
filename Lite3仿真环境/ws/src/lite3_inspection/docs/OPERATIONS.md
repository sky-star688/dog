# 双狗系统操作与故障排查

本文件覆盖 Dog 1（建图巡检）、Dog 2（拾取作业）、fleet_manager（协调器）的
启动顺序、验收阶段与常见故障。

## 1. 启动顺序

```
1) 底盘驱动 / 定位 / Nav2（两狗各自）
2) Dog 1：slam_toolbox → map_bridge → patrol_manager
3) Dog 1 冻结地图：/dog1/freeze_map
4) Dog 2：map_subscriber → dog2_localization → picking_manager
5) fleet_manager
6) Dog 1 开始巡逻：/dog1/start_patrol
```

Dog 2 的准入条件（缺一不可）：

- `/dog2/map_status` 中 `map_verified=true`（冻结地图哈希校验通过）
- `/dog2/localization_status=true`（AMCL 协方差收敛）

## 2. 分期验收

| 阶段 | 内容 | 通过条件 |
|------|------|----------|
| A | 各狗静态启动 | 话题、TF、相机、深度、`/cmd_vel` 单一发布者正常 |
| B | Dog 1 建图 + 冻结 | `/shared_map/metadata` 出现 frozen=true + sha256 |
| B2 | Dog 2 定位 | `map_verified=true` 且 `localization_status=true` |
| C | 协调器心跳 | `/fleet/status` 两狗 online=true |
| D | Dog 1 单航点巡逻 | 到点、暂停、恢复、停止均正常 |
| E | 机械臂台架 | 空载、dry_run，STOP 优先 |
| F | OpenMV 台架 | 只读 → 单应性标定 → 单次 KMS，默认 dry-run |
| G | 交接 dry-run | 目标上报 → 分配 → 接近 → 精定位 → 抓取 → 放置 → 撤退 |
| H | 完整抓取 | 仅在 A-G 全部通过且有人监护时 |

## 3. 常见故障

| 现象 | 优先检查 |
|------|----------|
| `start_patrol` 失败 | Nav2 `navigate_to_pose` 是否存在、`waypoints` 是否为空 |
| `map_verified` 始终 false | Dog 1 是否执行过 `freeze_map`；`/shared_map` 与 metadata 是否同一版本 |
| `localization_status` 始终 false | `picking.yaml` 的 `initial_x/y/yaw` 是否为测绘值；激光能否看到特征 |
| AMCL `cov_xy` 长期 >1.0 | 初始位姿错误或地图特征不足；调用 `/dog2/relocalize` |
| Dog 2 拒绝目标 | 未满足 `map_verified` + `localized` 双准入；查看 `picking_manager` 日志 |
| 交接超时 | `/fleet/status` 中 active_handoff；检查 operator 心跳与回传 |
| 机械臂不动 | `dry_run`、串口占用（OpenMV 工具不能与 arm_bridge 同占用）、组号配置 |
| `/cmd_vel` 多发布者 | Nav2 remap 是否指向 `/inspection/cmd_vel_nav` |
| 急停不生效 | `/dogN/estop` 与 fleet estop 传播；硬件急停优先 |

## 4. 安全边界

- 任一狗急停 → fleet_manager 广播 `/fleet/estop`，两狗停运动并发送机械臂 STOP。
- 巡检狗与占用作业区最小距离 5 m；两狗最小间距 3 m。
- 定位丢失时 Manager 立即关闭运动使能并暂停任务。
- 心跳超时 3 s 视为离线；交接超过 600 s 自动失败并释放作业区。
- 机械臂 `$DGS` 无完成 ACK：抓取/放置以实测忙窗时长为准，不把“写串口成功”当成动作完成。
