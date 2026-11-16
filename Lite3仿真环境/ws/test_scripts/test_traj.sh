#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3.launch.py gui:=false > /tmp/launch.log 2>&1 &
pid=$!

sleep 6

echo "===== nodes ====="
ros2 node list

echo "===== topic info /set_joint_trajectory ====="
ros2 topic info /set_joint_trajectory -v

echo "===== topic hz /set_joint_trajectory ====="
timeout 5 ros2 topic hz /set_joint_trajectory 2>&1 | head -5

echo "===== topic echo /set_joint_trajectory ====="
timeout 4 ros2 topic echo /set_joint_trajectory 2>&1 | head -30

echo "===== services ====="
ros2 service list | grep -iE 'joint|trajectory' || true

kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true
