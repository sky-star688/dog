# 接口契约

所有 JSON 均为 UTF-8 字符串，坐标为米、`map` 系；机械臂串口坐标不使用本接口。

## 1. 底盘安全接口（每狗独立前缀）

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/inspection/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 → mux | Nav2 controller 必须 remap 到这里 |
| `/inspection/cmd_vel_teleop` | `geometry_msgs/Twist` | 遥控 → mux | 优先级 100 |
| `/inspection/cmd_vel_aux` | `geometry_msgs/Twist` | 辅助避障 → mux | 优先级 20 |
| `/inspection/cmd_vel_raw` | `geometry_msgs/Twist` | mux → guard | 仲裁结果 |
| `/cmd_vel` | `geometry_msgs/Twist` | guard → 底盘 | 唯一速度出口 |
| `/dog1/motion_enable`、`/dog2/motion_enable` | `std_msgs/Bool` | manager → mux/guard | false 输出零 |
| `/dog1/estop`、`/dog2/estop` | `std_msgs/Bool` | 急停 → 全链 | true 锁存 |
| `/inspection/mux_status` | `std_msgs/String` | mux | JSON 诊断 |
| `/inspection/guard_status` | `std_msgs/String` | guard | JSON 诊断 |

限幅：`linear.x ±0.30`、`linear.y ±0.15`、`linear.z 0`、`angular.x/y 0`、
`angular.z ±0.50`。

## 2. 地图共享（冻结协议）

| 话题/服务 | 类型 | 说明 |
| --- | --- | --- |
| `/shared_map` | `nav_msgs/OccupancyGrid` | 冻结快照 |
| `/shared_map/metadata` | `std_msgs/String` | `{frozen, version, sha256, resolution_m, width, height, origin_x, origin_y}` |
| `/dog1/freeze_map` | `std_srvs/srv/Trigger` | 冻结当前地图并计算哈希 |
| `/dog1/unfreeze_map` | `std_srvs/srv/Trigger` | 解冻，恢复实时转发 |
| `/dog1/map_snapshot` | `std_srvs/srv/Trigger` | 返回快照元数据 JSON |
| `/dog2/map_status` | `std_msgs/String` | `{map_verified, map_frozen, version, expected_sha256, ...}` |

Dog 2 仅在哈希校验通过后把地图转发到本地 `/map`。

## 3. Dog 2 定位（AMCL）

| 话题/服务 | 类型 | 说明 |
| --- | --- | --- |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | dog2_localization 发布 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 收敛监控输入 |
| `/dog2/localization_status` | `std_msgs/Bool` | 收敛后 true |
| `/dog2/localization_info` | `std_msgs/String` | `cov_xy`、`cov_yaw`、`convergence_count` |
| `/dog2/set_initial_pose` | Trigger | 重发初始位姿 |
| `/dog2/relocalize` | Trigger | 全局重定位（放大协方差） |

## 4. 感知

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 2D 检测输入 |
| `/dog1/detections` | `std_msgs/String` | 2D 检测 JSON |
| `/dog1/detections_3d` | `std_msgs/String` | 含 `point_map` 与拒绝原因 |
| `/dog2/openmv_position` | `std_msgs/String` | OpenMV 精定位 |

`projection_status` 取值：`ready`、`no_valid_depth`、`depth_unavailable`、
`depth_stale`、`camera_info_unavailable`、`insufficient_depth`、
`depth_out_of_range`、`unstable_depth`、`transform_unavailable` 等。

## 5. 机械臂

`/dog2/arm_command`（`std_msgs/String`）只接受 `HOME`、`STOP`、`PICK`、`PLACE`
（大小写不敏感），映射为 `$DJR!`、`$DST!`、`$DGS:<pick_group>!`、
`$DGS:<place_group>!`。`$DGS` 无完成 ACK，忙窗时长必须现场实测。

## 6. 编队接口

| 话题/服务 | 说明 |
| --- | --- |
| `/fleet/heartbeat/{robot_id}` | 每狗心跳 JSON |
| `/fleet/target_detection` | 巡检狗目标上报 |
| `/{robot_id}/target_assignment` | 协调器分配目标给作业狗 |
| `/fleet/handoff_status` | 作业狗交接进度上报 |
| `/fleet/status` | 编队聚合状态 JSON |
| `/fleet/stop_all`、`/fleet/emergency_stop` | 双狗停止/急停 |
| `/fleet/generate_report` | 编队报告 |

`fleet_interfaces` 包同时提供等价的 msg/srv/action 定义，供强类型接入使用。

## 7. 兼容性

- 目标 ROS 2 Foxy、Python 3.8。
- 机械臂固件只接受 ASCII，不存在 JSON 协议。
- 保持“唯一 `/cmd_vel` 发布者”和“机械臂 STOP 优先”。
