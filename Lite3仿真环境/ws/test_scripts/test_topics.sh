#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3.launch.py gui:=false > /tmp/launch.log 2>&1 &
pid=$!

sleep 10

echo "===== topics ====="
ros2 topic list

echo "===== joint_states rate ====="
timeout 5 ros2 topic hz /joint_states 2>&1 | head -5

echo "===== joint_states sample ====="
timeout 4 ros2 topic echo /joint_states 2>&1 | head -40

echo "===== scan rate ====="
timeout 5 ros2 topic hz /scan 2>&1 | head -5

echo "===== imu rate ====="
timeout 5 ros2 topic hz /imu/data 2>&1 | head -5

echo "===== depth points info ====="
ros2 topic info /camera/points
timeout 5 ros2 topic hz /camera/points 2>&1 | head -5

echo "===== tf frames ====="
timeout 5 ros2 run tf2_ros tf2_echo TORSO D435I_LINK 2>&1 | head -20

kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true

echo "===== launch tail ====="
tail -30 /tmp/launch.log
