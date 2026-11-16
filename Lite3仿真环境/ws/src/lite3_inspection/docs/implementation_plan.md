# 实施计划（已完成 / 待办）

## 已完成

- [x] 单狗安全链：cmd_vel_mux → cmd_vel_guard，唯一 `/cmd_vel` 出口
- [x] 定位监控：里程计 + TF 就绪判断
- [x] 感知链：2D 检测 → 深度投影 → 拒绝原因
- [x] 机械臂 ASCII 桥接与语义动作
- [x] 事件存储（JSONL + fsync + artifact 哈希）
- [x] 双狗拆分：lite3_01_mapping / lite3_02_picking
- [x] 地图冻结协议 + SHA256 校验
- [x] Dog 2 AMCL 初始位姿 + 收敛准入 + 全局重定位
- [x] fleet_manager：心跳/目标分配/作业区预约/串行交接/急停/报告
- [x] 分狗编译与启动脚本
- [x] 领域层测试

## 待办

- [ ] 现场测绘：patrol_mission.yaml / fleet.yaml / picking.yaml 坐标
- [ ] 动作组录制与忙窗实测
- [ ] OpenMV 现场标定并接入 ROS
- [ ] 并行巡检模式（Dog 1 不等交接）
- [ ] Gazebo/Isaac 仿真回归
- [ ] 远程地面站 Web 视图
