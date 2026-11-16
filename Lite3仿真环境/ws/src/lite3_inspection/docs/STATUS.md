# 当前状态

最后更新：2026-11-11

## 代码状态

| 组件 | 状态 |
| --- | --- |
| lite3_inspection（共享库/遗留栈） | 可编译，领域测试覆盖 |
| lite3_01_mapping（建图巡检狗） | 节点与 launch 就绪 |
| lite3_02_picking（拾取作业狗） | 节点与 launch 就绪 |
| fleet_manager（协调器） | 节点与 launch 就绪 |
| fleet_interfaces | msg/srv/action 定义就绪（需 colcon 编译） |
| 脚本 | build/test/preflight/run_dog1/run_dog2/run_fleet/run_dual |

## 未完成

- 现场坐标测绘（航点、作业区、Dog 2 初始位姿）
- 动作组录制与忙窗实测
- OpenMV 现场标定
- 双狗真机联调（尚未执行）

## 验证记录

- Python 全量 `compileall` 通过
- 领域层冒烟测试通过（交接生命周期、作业区预约、巡逻/拾取状态机）
- 未在硬件上运行；不得宣称实机通过
