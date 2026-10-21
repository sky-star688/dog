# Lite3 工业巡检 + STM32 机械臂 ROS 2 框架

这是面向绝影 Lite3 感知主机的 ROS 2 Foxy 工作空间。它已经按所给手册和源码中的真实接口搭好骨架，但第一次运行默认**不会驱动机械臂，也不会自动抓取**：

- Lite3 运动使用官方 `transfer_ros2` 服务和 `/cmd_vel` (`geometry_msgs/msg/Twist`)。
- 相机使用官方 `realsense_ros2` 服务，默认订阅 `/camera/color/image_raw`。
- MyArm STM32 使用 115200、8N1 串口和原固件 ASCII 指令。
- 视觉推理支持 Ultralytics `.pt`，也支持不依赖 PyTorch 的 YOLOv8 风格 `.onnx`。
- 发现目标后先停狗并等待人工确认；`auto_pick` 默认是 `false`。

> 重要：这不是完成标定的成品产线程序。相机到机械臂的外参、夹爪动作组、抓取距离、工位地图和异常类别都必须在现场标定。首次测试必须架空机械臂或拆掉夹爪负载，并保证机器狗周围至少 2 m 无人、无易损物。

## 1. 系统结构

```text
RealSense D435i -> /camera/color/image_raw -> vision_detector
                                             |
                                             v
                                  /inspection/detections
                                             |
Nav2/巡检逻辑 -> /inspection/cmd_vel_raw -> cmd_vel_guard -> /cmd_vel
                                             |                |
                                             |                v
                                             |        Lite3 transfer_ros2 -> 运动主机
                                             v
                                    inspection_manager
                                             |
                                      /arm/command
                                             |
                                      arm_serial_bridge
                                             |
                                      USB-UART -> STM32 MyArm
```

节点职责：

- `cmd_vel_guard`：限速、250 ms 指令超时归零、显式启停。该超时时间与 Lite3 手册中的运动超时一致。
- `vision_detector`：图像推理，发布 JSON 检测结果与标注图。
- `arm_serial_bridge`：把 `HOME/PICK/PLACE/STOP` 转成 STM32 指令。
- `inspection_manager`：监控稳定目标，先停运动，再要求人工确认抓取。

## 2. 先解决“连接机器狗后双方都不能上网”

机器狗 Wi-Fi 是设备局域网，不应承担互联网默认路由。最稳定的方案是保留机器狗 Wi-Fi，再给 Windows 和感知主机增加第二条上网链路。

### 方案 A：Windows 双网卡（推荐）

1. Windows 的 Wi-Fi 继续连接机器狗，用于 NoMachine/SSH，目标地址是 `192.168.1.103`。
2. Windows 同时通过有线网、另一个 USB Wi-Fi 网卡或手机 USB 共享上网。
3. 打开 `控制面板 -> 网络连接 -> 机器狗 Wi-Fi -> IPv4 -> 高级`，不要为机器狗链路手工填互联网 DNS；若能配置，取消该链路的默认网关，或把它的接口跃点数设为较大值（例如 50）。
4. 给真正上网的有线/USB 链路保留默认网关，并设较小跃点数（例如 10）。
5. 在 PowerShell 验证：

```powershell
route print
ping 192.168.1.103
ping 1.1.1.1
```

能同时 ping 通机器狗和公网就正确。不要把 Windows 的 Internet Connection Sharing 绑定到机器狗 Wi-Fi；ICS 常会改写网段，导致 NoMachine 失联。

### 方案 B：让感知主机直接上网

给感知主机接入一条与 `192.168.1.0/24` **不冲突**的链路：网线接路由器、USB 网卡或手机 USB 共享。然后在机器狗终端检查：

```bash
ip -br address
ip route
ping -c 3 1.1.1.1
ping -c 3 www.baidu.com
```

正确状态是：机器狗内部通信仍走 `192.168.1.x`，互联网默认路由走新增网卡。不要改 `192.168.1.103`，否则雷达、运动主机和 NoMachine 可能失联。

### 方案 C：完全离线部署（最安全）

本框架就是按此方案准备的。训练、下载依赖都在有网 Windows 完成，再通过机器狗 Wi-Fi 使用 `scp` 或 NoMachine 共享目录传入。若需要 Ubuntu 包，可在一台 Ubuntu 20.04/ARM64 环境下载 `.deb`；不要把 Windows x64 Python wheel 复制到 RK3588/ARM64 上安装。

## 3. 复制到机器狗并编译

将整个文件夹复制到感知主机 `/home/ysc/lite3_industrial_inspection_ws`。在 Windows PowerShell 可用：

```powershell
scp -r .\lite3_industrial_inspection_ws ysc@192.168.1.103:/home/ysc/
```

在机器狗终端执行：

```bash
cd /home/ysc/lite3_industrial_inspection_ws
chmod +x scripts/*.sh
cd /home/ysc/scripts
sudo ./print_ros_version.sh
```

若不是 ROS 2：

```bash
cd /home/ysc/scripts
sudo ./switch_ros_version.sh ros2
sudo reboot
```

重连后编译：

```bash
cd /home/ysc/lite3_industrial_inspection_ws
./scripts/robot_setup.sh
source install/setup.bash
```

Lite3 手册对应的是 ROS 2 Foxy。Foxy 已停止上游维护，因此不要在机器狗上擅自升级 ROS 发行版；厂商的 `transfer_ros2`、导航和相机包都按当前镜像配套。

## 4. 第一次只测试机器狗运动

### 4.1 检查官方通信服务

```bash
sudo systemctl status transfer_ros2
sudo systemctl start transfer_ros2
journalctl -fu transfer_ros2
```

新开终端：

```bash
source /opt/ros/foxy/setup.bash
ros2 topic list | grep -E 'cmd_vel|leg_odom|imu/data|joint_states'
ros2 topic echo /imu/data
```

应看到至少 `/cmd_vel`、`/leg_odom`、`/imu/data`、`/joint_states`。如果没有，不要继续运动测试，先查 `transfer_ros2` 日志。

### 4.2 官方接口低速点动

1. 把机器狗放到空旷平地。
2. 用 APP/手柄让机器狗起立，并切到“自主模式”。
3. 保持手放在软急停位置。
4. 发布 0.05 m/s，持续 2 秒后按 `Ctrl+C`：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

立即停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

坐标约定：`linear.x` 向前为正，`linear.y` 向左为正，`angular.z` 左转为正。手册给出的底层最大范围更大，但本框架默认限制为 0.30 m/s、0.15 m/s、0.50 rad/s。

### 4.3 通过安全门测试

启动框架后，先启用安全门，再向原始话题发命令：

```bash
ros2 topic pub --once /inspection/motion_enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 10 /inspection/cmd_vel_raw geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

超过 250 ms 未收到新命令时，安全门持续发布零速度。

## 5. 机械臂连接和动作组

STM32 原固件确认使用 115200、8 数据位、无校验、1 停止位，并识别以下命令：

- `$DST!`：全部舵机停止。
- `$DJR!`：全部舵机复位。
- `$DGS:1!`：执行动作组 1。
- `$DGT:1-3,1!`：动作组 1 到 3 执行 1 次。
- `$KMS:x,y,z,time!`：末端坐标运动，坐标单位 mm，时间单位 ms。
- `#000P1500T1000!`：0 号舵机到 PWM 1500，用时 1000 ms。

推荐使用带隔离的 USB-UART，确认电平是 3.3 V TTL，共地；不要把 USB-UART 的 5 V 接到机器人电源。机械臂舵机必须使用独立、足够电流的电源，不能从感知主机 USB 口给舵机供电。

连接后：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
sudo usermod -aG dialout ysc
```

重新登录后，将 `config/inspection.yaml` 的 `port` 改为实际设备，并优先配置 udev 固定名。示例见 `udev/99-myarm.rules.example`。

第一次必须保留：

```yaml
dry_run: true
```

启动后测试映射但不发送串口：

```bash
ros2 topic echo /arm/status
ros2 topic pub --once /arm/command std_msgs/msg/String "{data: 'HOME'}"
ros2 topic pub --once /arm/command std_msgs/msg/String "{data: 'GROUP:1'}"
ros2 service call /arm/stop std_srvs/srv/Trigger "{}"
```

确认输出正确，架空机械臂，再改为 `dry_run: false`。先测试 `STOP`，再测试 `HOME`。最后在机械臂上位机中分别录制：

- 动作组 1：预抓取位 -> 下探 -> 夹紧 -> 抬起。
- 动作组 2：移动到放置位 -> 松开 -> 回安全位。

如果实际组号不同，只修改 `pick_command` 和 `place_command`。

## 6. 相机和视觉训练

### 6.1 检查相机

```bash
sudo systemctl status realsense_ros2
sudo systemctl start realsense_ros2
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```

若实际彩色话题不同，修改 `camera_topic`。可用以下方式预览：

```bash
ros2 run rqt_image_view rqt_image_view
```

### 6.2 在 Windows 采集和标注

不要只拍“干净物块”。数据至少覆盖：不同光照、角度、距离、背景、遮挡、机器狗运动模糊，以及大量正常场景负样本。建议第一轮每类 500 张以上，按现场、日期或视频段划分训练/验证集，避免相邻视频帧同时落入训练和验证集造成虚高。

建议类别拆分：

- `anomaly`：漏液、异物、表计异常等只报警目标。
- `block`：允许抓取的物块。
- 如果不同物块需要不同动作，拆成 `block_red`、`block_blue` 等，并同步修改配置。

YOLO 数据目录示例：

```text
dataset/
  images/train
  images/val
  labels/train
  labels/val
  data.yaml
```

`data.yaml`：

```yaml
path: C:/data/lite3_inspection
train: images/train
val: images/val
names:
  0: anomaly
  1: block
```

在有网 Windows 的 Python 环境安装 Ultralytics，然后训练并导出 ONNX：

```powershell
python -m pip install ultralytics
yolo detect train model=yolov8n.pt data=C:\data\lite3_inspection\data.yaml epochs=100 imgsz=640 device=0
yolo export model=runs\detect\train\weights\best.pt format=onnx imgsz=640 opset=12 simplify=False
```

把 `best.onnx` 复制到机器狗的 `models/`，并确保 `models/labels.txt` 的顺序完全一致。先在录制视频上评估漏检和误检，再进行机器人静止测试，最后才做低速巡检。

### 6.3 检测输出格式

`/inspection/detections` 是 `std_msgs/String`，内容为 JSON：

```json
{
  "frame_id": "camera_color_optical_frame",
  "detections": [
    {"class_id": 1, "class_name": "block", "confidence": 0.91,
     "bbox": [100, 80, 240, 260], "center": [170, 170]}
  ]
}
```

注意：二维框中心不能直接当作机械臂抓取坐标。正式抓取需要彩色-深度对齐、相机内参反投影，以及 `camera_link -> arm_base` 外参标定。未完成这三项前，只能使用固定工位、固定抓取位的动作组。

## 7. 启动完整框架

先编辑：

```text
src/lite3_inspection/config/inspection.yaml
```

至少确认模型绝对路径、相机话题、串口、动作组、目标类别。然后：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/lite3_industrial_inspection_ws/install/setup.bash
ros2 launch lite3_inspection inspection.launch.py
```

另开终端观察：

```bash
ros2 topic echo /inspection/event
ros2 topic echo /arm/status
ros2 topic hz /inspection/detections
```

开始监控：

```bash
ros2 service call /inspection/start std_srvs/srv/Trigger "{}"
```

稳定识别目标后，框架会关闭运动安全门。确认机器狗已停止、机械臂与物块对齐后：

```bash
ros2 service call /inspection/confirm_pick std_srvs/srv/Trigger "{}"
```

到达放置工位且再次确认停止后：

```bash
ros2 service call /inspection/place std_srvs/srv/Trigger "{}"
```

任何时候停止：

```bash
ros2 service call /inspection/stop std_srvs/srv/Trigger "{}"
```

## 8. 接入 Lite3 自带导航

自带 ROS 2 导航启动方式来自感知手册：

```bash
cd ~/lite_cog_ros2/system/scripts/lidar
./start_lslidar.sh        # 雷神 C16
# 或 ./start_livox.sh     # Livox

cd ~/lite_cog_ros2/system/scripts/nav
./start_nav.sh
```

首次应先按手册建图、保存地图并在 RViz 初始化定位。多点巡检可参考 `~/lite_cog_ros2/pipeline/src/pipeline/LocationRecorder.py` 和 `Task.py`。

集成本框架时需要确保 Nav2 的速度先经过安全门：把 Nav2 输出从 `/cmd_vel` 重映射为 `/inspection/cmd_vel_raw`，安全门再输出 `/cmd_vel`。如果 Nav2 仍直接发布 `/cmd_vel`，检测到异常时它会绕过安全门，这是不安全的。

推荐状态机：

```text
建图/标点 -> 导航到巡检点 -> 停稳 -> 拍照/检测
    -> 正常：下一点
    -> 异常但不可抓：记录图片、位姿、时间并报警
    -> 可抓物块：关闭导航 -> 二次确认定位 -> 抓取 -> 导航到放置点 -> 放置
    -> 任一超时/通信中断：零速度 + $DST! + 人工接管
```

## 9. 从固定动作组升级到视觉抓取

按以下顺序做，不要一步到位：

1. 固定机器狗、固定物块，验证动作组抓取 100 次。
2. 使用 AprilTag/标定板求 `camera_link -> arm_base` 外参。
3. 启用 D435i 对齐深度；对检测框中心附近取深度中位数，不用单个像素。
4. 将相机三维点通过 TF2 转到 `arm_base`。
5. 做工作空间、桌面高度、关节极限、碰撞和夹爪电流检查。
6. 只下发“预抓取位”，人工确认后再下探和夹紧。
7. 连续验证后才考虑把 `auto_pick` 改为 `true`。

## 10. 验收清单

- 断开视觉节点时，机器狗在 250 ms 内停止。
- `/inspection/stop` 能让机器狗零速并发送 `$DST!`。
- 串口拔出、相机断开、模型缺失都只报错，不产生运动。
- 自主模式关闭后，手柄可立即接管。
- 机械臂独立供电，急停能切断舵机动力。
- 正常场景误抓取率为 0；误检只能报警，不能直接执行抓取。
- 每次任务记录时间、位姿、原图、检测结果、抓取结果和故障原因。

## 11. 常见问题

`/cmd_vel` 有数据但狗不动：检查 `transfer_ros2`、是否起立、APP 是否为自主模式，并查看 `journalctl -fu transfer_ros2`。

相机没话题：检查 `realsense_ros2`，再用 `ros2 topic list | grep -E 'image|camera'` 找真实话题。

串口 Permission denied：确认用户属于 `dialout`，重新登录，或安装 udev 规则；不要长期用 `sudo ros2 launch`。

ONNX 输出维度报错：模型可能不是 YOLOv8 风格检测模型。先用 `.pt` + Ultralytics 验证，或按实际输出修改 `infer_onnx()`。

NoMachine 很卡：建图、TensorRT 推理时关闭 RViz 和 NoMachine 画面，使用 SSH 看日志，减少 GPU/桌面占用。

## 12. 依据的本地资料

- 《绝影Lite3感知开发手册(beta) V2.2.2》：ROS 2 切换、`transfer_ros2`、`realsense_ros2`、`/cmd_vel`、建图导航。
- 《绝影Lite3运动主机通讯接口(beta) V1.0.7》：UDP 端口 43893、心跳和速度范围；本框架优先使用厂商 ROS 2 转换服务，不重复实现 UDP。
- `lite_cog_ros2`：现有导航、跟踪和 pipeline 工程结构。
- `视觉机械臂MyArm-2`：STM32 串口参数与 `$DGS/$DJR/$DST/$KMS` 命令解析。

外部参考：[Ultralytics Train](https://docs.ultralytics.com/modes/train)、[Ultralytics Export](https://docs.ultralytics.com/modes/export) 和 [ROS 2 Foxy EOL 说明](https://docs.ros.org/en/foxy/Releases/End-of-Life.html)。Foxy 已 EOL，但 Lite3 厂商镜像仍基于它，因此本项目保持兼容，不在机器人上直接升级发行版。
