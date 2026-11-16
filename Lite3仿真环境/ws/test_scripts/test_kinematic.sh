#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3_kinematic.launch.py gui:=false > /tmp/launch.log 2>&1 &
pid=$!

sleep 8

echo "===== topics ====="
ros2 topic list

echo "===== odom before ====="
timeout 4 ros2 topic echo /odom 2>&1 | head -25

echo "===== command forward ====="
timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" >/dev/null 2>&1 || true

sleep 1

echo "===== odom after ====="
timeout 4 ros2 topic echo /odom 2>&1 | head -25

echo "===== scan ====="
timeout 3 ros2 topic echo /scan 2>&1 | head -12

echo "===== imu orientation ====="
timeout 3 ros2 topic echo /imu/data 2>&1 | sed -n '1,15p'

kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true

echo "===== launch tail ====="
tail -30 /tmp/launch.log
