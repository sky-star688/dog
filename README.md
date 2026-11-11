# dog — 双 Lite3 协同巡检与作业系统

两条 DeepRobotics Lite3 机器狗在同一张已知地图、同一 ROS 2 域下完成：

> 巡检狗发现目标 → 协调器分配 → 作业狗精定位抓取 → 巡检狗继续巡检 → 双狗安全返回并生成报告

## 两只狗，两套硬件，一张共享地图

```
┌──────────────────────────────────────────────────────────────────┐
│                        fleet_manager                              │
│                     (地面站协调器，一台主机)                         │
│   心跳监控 │ 目标分配 │ 作业区预约 │ 交接握手 │ 急停传播 │ 报告生成     │
└──────┬────────────────────────────────────────────┬──────────────┘
       │                                            │
┌──────▼──────────────────────────┐  ┌─────────────▼──────────────────┐
│       Dog 1: lite3_01           │  │      Dog 2: lite3_02           │
│       建图 + 巡检狗              │  │      拾取 + 作业狗              │
│                                 │  │                                │
│  📡 LiDAR (Livox MID360)        │  │  📷 OpenMV (精定位相机)         │
│  📷 深度相机 (RealSense D435)    │  │  📷 深度相机 (RealSense D435)   │
│                                 │  │  🦾 MyArm-2 机械臂             │
│  ─────────────────────────      │  │  ──────────────────────────     │
│  🗺️  SLAM 建图 + 冻结快照        │  │  🗺️  冻结地图 + AMCL 定位       │
│  🚶  航点巡检(Nav2)             │  │  🚶  导航至接近位(Nav2)         │
│  👁️  异常检测(YOLO+深度)        │  │  🎯  OpenMV 精定位             │
│  📡  目标上报(fleet_manager)    │  │  🦾  机械臂抓取/放置            │
│  ⏸️  等待交接完成               │  │  📡  完成上报(fleet_manager)    │
│  ▶️  继续巡检                   │  │  🏠  撤退至安全位              │
└─────────────────────────────────┘  └────────────────────────────────┘

         共享地图: /shared_map (Dog 1 冻结快照 → Dog 2 订阅 + 哈希校验)
```

## 仓库结构

```
dog/
├── Lite3工业巡检ROS2包/            # ROS2 工作空间（所有包都在这里）
│   ├── src/
│   │   ├── fleet_interfaces/      # 自定义 msg/srv/action
│   │   ├── lite3_inspection/      # 共享库：geometry/safety/robots/arm/fleet_client
│   │   ├── fleet_manager/         # 协调器
│   │   ├── lite3_01_mapping/      # 🗺️ Dog 1 – LiDAR SLAM + 深度相机 + 巡检
│   │   └── lite3_02_picking/      # 🦾 Dog 2 – OpenMV + 深度相机 + MyArm-2
│   ├── scripts/                   # build.sh / run_dog1.sh / run_dog2.sh / run_fleet.sh / run_dual.sh
│   ├── models/                    # YOLO 模型
│   └── README.md                  # 工作空间详细说明（含定位漂移处理）
├── 机器狗源文件/                   # 第三方驱动（Livox、Leishen、Orbbec、MID360）
└── 视觉机械臂MyArm-2/              # MyArm-2 固件与工具
```

## 快速开始

```bash
cd Lite3工业巡检ROS2包

bash scripts/build.sh          # 编译全部
bash scripts/build.sh dog1     # 只编译 Dog 1
bash scripts/build.sh dog2     # 只编译 Dog 2
bash scripts/build.sh fleet    # 只编译协调器

bash scripts/run_dual.sh       # 一键启动双狗系统
```

## 地图漂移与 Dog 2 初始定位

Dog 1 实时建图时地图会变化，Dog 2 若直接在上面定位必然漂移。系统用**冻结协议**解决：

1. Dog 1 建图完成后执行 `/dog1/freeze_map`，发布带 SHA256 的静态地图快照
2. Dog 2 只接受冻结且哈希校验通过的地图
3. Dog 2 的初始位姿必须在 `config/picking.yaml` 中按测绘值填写，AMCL 收敛后才接受目标
4. 丢失定位时 `/dog2/relocalize` 可全局重定位

详见 [Lite3工业巡检ROS2包/README.md](Lite3工业巡检ROS2包/README.md)。

## 许可

MIT License
