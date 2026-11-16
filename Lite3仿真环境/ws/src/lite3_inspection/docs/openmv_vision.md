# OpenMV 视觉链路

## 协议

OpenMV 通过 USB VCP 只发送目标遥测：

```
@OVM:<cx>,<cy>,<width>,<height>,<pixels>!
```

这是主机侧帧，不是厂商固件帧；OpenMV 从不写机械臂。

## 主机工具

| 工具 | 作用 | 默认安全 |
| --- | --- | --- |
| `scripts/openmv_monitor.py` | 只读监视，可选 `--homography` 换算 | 不打开机械臂串口 |
| `scripts/openmv_calibrate.py` | 离线拟合像素→基座平面单应性 | 不打开串口 |
| `scripts/openmv_arm_goto.py` | 单次 `$KMS` 台架 | dry-run，`--execute` 需输入 SEND |

## ROS 链路（Dog 2）

```
OpenMV USB → 主机桥接 → /openmv/telemetry (String)
  → openmv_positioner → /dog2/openmv_position (JSON)
  → picking_manager（稳定帧数达标后执行 PICK）
```

`picking_manager` 要求 `openmv_stable_frames` 个连续稳定读数才推进；
默认 `mm_per_pixel` 与相机-机械臂偏移必须标定后填写。

## 标定步骤

1. 固定相机与机械臂，采集至少 4 个非共线像素点及其基座坐标（mm）。
2. 运行 `openmv_calibrate.py`，残差需低于 `--max-rms-mm`（建议 2.0）。
3. 标定 JSON 写入 `workspace_mm` 边界后再允许 `--execute`。
4. 首次只抬到目标上方，保持 `dry_run`，人工确认到达。
