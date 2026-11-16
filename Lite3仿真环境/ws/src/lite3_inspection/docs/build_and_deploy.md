# 构建与部署

## 环境

- Ubuntu 20.04、ROS 2 Foxy、Python 3.8
- colcon、rclpy、cv_bridge、OpenCV、numpy、pyyaml、pyserial、tf2_ros

## 构建

```bash
cd Lite3工业巡检ROS2包

# 全部
bash scripts/build.sh

# 单狗（独立调试）
bash scripts/build.sh dog1
bash scripts/build.sh dog2
bash scripts/build.sh fleet

source install/setup.bash
```

`build.sh` 不修改 `~/.bashrc`，不以 root 运行。

## 部署到两台机器狗

1. 两台狗的 ROS 域保持一致（`ROS_DOMAIN_ID`），或通过网桥互通。
2. 每台机器只跑自己那条狗的 launch；`fleet_manager` 跑在地面站。
3. 地图由 Dog 1 生成并冻结；Dog 2 依赖 `/shared_map`，不本地建图。
4. `picking.yaml` 的 `initial_x/y/yaw` 与作业区坐标必须按测绘值填写。

## 启动

```bash
# 地面站
bash scripts/run_fleet.sh

# Dog 1 机器
bash scripts/run_dog1.sh

# Dog 2 机器
bash scripts/run_dog2.sh
```

## 回滚

- 停止顺序：`/fleet/stop_all` → 停 launch → 停 Nav2/定位/相机/底盘 → 机械臂断电。
- 任何一次失败先回滚到已验证的配置与动作组，不要靠调参“蒙过”。
