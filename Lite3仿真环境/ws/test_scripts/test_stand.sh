#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3.launch.py gui:=false > /tmp/launch.log 2>&1 &
pid=$!

sleep 8

echo "===== joint_states ====="
timeout 4 ros2 topic echo /joint_states 2>&1 | head -35

echo "===== scan info ====="
ros2 topic info /scan
echo "===== scan sample ====="
timeout 4 ros2 topic echo /scan 2>&1 | head -18

echo "===== imu info ====="
ros2 topic info /imu/data
echo "===== imu sample ====="
timeout 4 ros2 topic echo /imu/data 2>&1 | head -18

echo "===== depth info ====="
ros2 topic info /camera/points

echo "===== tf sample ====="
timeout 4 ros2 topic echo /tf 2>&1 | head -20

kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true

echo "===== launch tail ====="
tail -35 /tmp/launch.log
