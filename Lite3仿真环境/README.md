# Lite3 仿真环境

这是 `dog` 仓库的 Lite3 Gazebo Classic 仿真环境，基于 **ROS 2 Foxy + Gazebo 11**，运行在 Docker 里。宿主机不需要安装 ROS 2，也不需要重装系统。

当前环境用于验证：

- Lite3 模型加载、TF、关节状态、IMU、2D LiDAR、深度点云
- 运动学底座（kinematic base）下的 `/cmd_vel` → `/odom` → Nav2/AMCL 链路
- MyArm-2 简化机械臂模型与 `/arm/command` 动作组联动
- 仓库 `Lite3工业巡检ROS2包` 的 `cmd_vel_guard`、`arm_serial_bridge`、`inspection_manager`
- 后续两狗、静态地图、深度定位、VLM 目标识别的集成

## 目录

```text
Lite3仿真环境/
  Dockerfile
  build.sh              # 构建 Docker 镜像
  build_ws.sh           # 在容器里 colcon build
  setup_repo_ws.sh      # 把 ../Lite3工业巡检ROS2包/src/lite3_inspection 复制进工作空间
  sim.sh                # GUI 仿真
  sim_headless.sh       # 无界面仿真（Xvfb）
  run.sh                # 进入带 ROS 环境的容器 shell
  exec.sh               # 对运行中的容器另开 shell
  stop.sh               # 停止容器
  ws/
    src/lite3_sim/         # Lite3 URDF、launch、Gazebo world、机械臂模型、仿真节点
    src/lite3_inspection/  # 从 ../Lite3工业巡检ROS2包 同步的仓库代码
    src/deep_robotics_model/ # 官方模型 git submodule（可选，仿真已内置 Lite3 网格）
    test_scripts/          # 冒烟测试脚本
```

## 为什么仓库里不是“整个 6.48GB 仿真镜像”

GitHub 仓库只放**可重建的源码**，不放下面这些：

- `ws/build`、`install`、`log`：`colcon` 生成物，路径相关，本地重建即可。
- `lite3_sim:foxy` Docker 镜像：约 6.48GB，应放在 Docker 镜像仓库，不是 Git。
- `deep_robotics_model` 官方模型仓库：约 533MB，已作为 **git submodule** 放在 `ws/src/deep_robotics_model`；主仓库只记录 commit 指针。
- `Lite3_rl_deploy` 官方控制器仓库：约 185MB，动态步态/RL 部署时再拉取。

`ws/src/lite3_inspection` 保留了一份同步副本，`build_ws.sh` 会从主仓库重新同步，方便直接 `colcon build`。

初始化 submodule：

```bash
git submodule update --init --recursive
```

需要官方控制器源码时：

```bash
./fetch_official_sources.sh
```

会拉取到 `third_party/`，该目录已被 `.gitignore` 忽略。

如果要把镜像也发布，应该推到 GitHub Container Registry 或 Docker Hub，而不是提交到 Git。

## 前置条件

- Ubuntu + Docker
- 当前用户在 `docker` 组，或愿意使用 `sudo docker`
- 有图形界面时可直接开 Gazebo；无图形界面用 `sim_headless.sh`（Xvfb）
- 可选：NVIDIA Container Toolkit（Gazebo Classic 用 CPU/Xvfb 也能跑，不强制）

如果 `docker ps` 报权限错误：

```bash
sudo usermod -aG docker "$USER"
# 重新登录，或临时使用 sudo docker
```

## 1. 构建镜像

```bash
cd Lite3仿真环境
./build.sh
```

默认基础镜像：

```text
docker.m.daocloud.io/osrf/ros:foxy-desktop
```

如果网络环境不同，可以改 `Dockerfile` 里的 `BASE_IMAGE`。

## 2. 准备仓库代码和工作空间

```bash
cd Lite3仿真环境
./build_ws.sh
```

`build_ws.sh` 会自动初始化 submodule 并调用 `setup_repo_ws.sh`。也可以单独执行：

```bash
./setup_repo_ws.sh
./build_ws.sh
```

`setup_repo_ws.sh` 会把：

```text
../Lite3工业巡检ROS2包/src/lite3_inspection
```

复制到：

```text
ws/src/lite3_inspection
```

`ws/src/lite3_inspection` 已提交一份副本；`setup_repo_ws.sh` 会从 `../Lite3工业巡检ROS2包` 重新同步并覆盖它，保证和主仓库代码一致。

## 3. 启动仿真

**GUI + 运动学底座（推荐先跑这个）**

```bash
./sim.sh
```

**无界面 + 运动学底座**

```bash
./sim_headless.sh
```

**动态物理模型（会倒，暂未接平衡控制器）**

```bash
./sim.sh dynamic
```

动态模式用于后续接官方 `Lite3_rl_deploy`/`sdk_deploy` 控制器，当前不要用它跑 Nav2。

## 4. 验证话题

另开一个容器 shell：

```bash
./exec.sh
```

然后：

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /scan
ros2 topic echo /imu/data
ros2 topic echo /camera/points
ros2 topic echo /odom
```

发送速度：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 5. 运行仓库节点

先启动仿真，再在另一个容器 shell 里：

```bash
ros2 launch lite3_sim repo_bridge.launch.py
```

这会启动：

- `cmd_vel_guard`：`/inspection/cmd_vel_raw` → `/cmd_vel`
- `arm_serial_bridge`：`dry_run: true`，只验证 `/arm/command` 映射
- `inspection_manager`：`auto_pick: false`

测试流程：

```bash
ros2 service call /inspection/start std_srvs/srv/Trigger "{}"
ros2 topic pub --rate 10 /inspection/cmd_vel_raw geometry_msgs/msg/Twist \
"{linear: {x: 0.2}}"
```

发布一个假检测：

```bash
ros2 topic pub --once /inspection/detections std_msgs/msg/String \
"{data: '{\"detections\":[{\"class_name\":\"block\",\"confidence\":0.9}]}'}"
```

确认抓取：

```bash
ros2 service call /inspection/confirm_pick std_srvs/srv/Trigger "{}"
```

`inspection_manager` 会发布 `/arm/command: PICK`，仿真里的机械臂会切到预抓取位。

## 6. 进入/退出 Docker

**进入**

```bash
./run.sh      # 新建并进入容器
./exec.sh     # 已经在运行时，另开一个 shell
```

**退出**

```bash
exit          # 退出容器 shell
```

**停止容器**

```bash
./stop.sh
```

GUI 仿真按 `Ctrl+C` 结束 `ros2 launch`，再 `exit` 退出容器。

## 7. 机械臂模型说明

- 这是一个**简化 MyArm-2 模型**，不是精确 CAD。
- 连杆长度参考固件：`L0=111 mm`、`L1=105 mm`、`L2=86 mm`、`L3=175 mm`。
- 关节：底座旋转、肩、肘、腕、腕 roll、左右夹爪。
- `arm_pose.py` 同时发布腿和机械臂姿态，支持 `/arm/command`：
  - `HOME` / `STOP`
  - `PICK` / `GROUP:1`
  - `PLACE` / `GROUP:2`
- 真实 MyArm-2 的 `KINEMATICS`/串口协议仍需在真机台架验证。

## 8. 已知限制

- 动态模型没有平衡控制器，会倒；运动学底座是漂浮底座，只适合 Nav2/AMCL/传感器测试。
- Gazebo 启动时可能出现 `Get noise index not valid` 警告，不影响 `/scan` 和 `/imu/data`。
- 深度点云话题是 `/camera/points`，不是 `/camera/depth/points`。
- 模型根 link 是 `TORSO`；Nav2 配置里需要把 `TORSO` 当 base，或后续加 `base_link`。
- 当前是单狗 + 机械臂；两狗命名空间、静态地图共享、B 的深度→scan→AMCL 还未完成。

## 9. 第三方资源

- Lite3 URDF/STL 来自 `DeepRoboticsLab/deep_robotics_model`，BSD-3-Clause。
- MyArm-2 厂商源码/手册/结构图纸来自本仓库 `视觉机械臂MyArm-2/`。
- 机械臂 Gazebo 模型是简化模型，不是厂商精确 CAD。

## 10. 分支

仿真材料随仓库提交到 `zwq1111` 分支。
