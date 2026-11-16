#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3_kinematic.launch.py gui:=false > /tmp/launch.log 2>&1 &
pid=$!

sleep 8

echo "===== tf echo TORSO -> arm_roll_link ====="
timeout 5 ros2 run tf2_ros tf2_echo TORSO arm_roll_link 2>&1 | head -25

echo "===== tf arm frames ====="
timeout 4 ros2 topic echo /tf 2>&1 | grep -E 'arm_|gripper_' | head -30

kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true
