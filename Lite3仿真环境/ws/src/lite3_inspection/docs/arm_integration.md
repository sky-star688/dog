# MyArm-2 机械臂集成

## 接线

- USB 串口：`/dev/ttyUSB0`，115200 8N1，无换行终止符。
- 供电与急停按厂商流程；首次保持 `dry_run: true`。
- udev 规则示例见 `udev/99-myarm.rules.example`。

## 语义动作

| ROS 动作 | 串口帧 | 备注 |
| --- | --- | --- |
| `HOME` | `$DJR!` | 回中值，不是已验证安全姿态 |
| `STOP` | `$DST!` | 始终允许 |
| `PICK` | `$DGS:<pick_group>!` | 组号未配置则拒绝 |
| `PLACE` | `$DGS:<place_group>!` | 组号未配置则拒绝 |

不支持 JSON、坐标或任意原始帧（`arm.py` 只校验 `$...!` 且有长度上限）。

## 动作组录制流程

1. 用厂商软件录 `pick`/`place` 两组，空载低速验证。
2. 确认组号后写入 `lite3_02_picking/config/picking.yaml` 的 `arm.pick_group/place_group`。
3. 先 dry-run：`/dog2/arm_command` 只打印帧。
4. 空载台架 + 急停可用，执行 `/dog2/home`、`/dog2/stop`、单次 PICK。
5. 实测忙窗时长，回填 `pick_timeout_sec` / `place_timeout_sec`。

## 已知限制

- 固件不返回通用完成 ACK，`action_window_elapsed` 只表示窗口结束。
- 串口不能被 `arm_bench.py`/`openmv_arm_goto.py` 与 `arm_bridge` 同时占用。
- `$KMS` 使用机械臂基座系毫米坐标，需单独的像素→基座标定。
