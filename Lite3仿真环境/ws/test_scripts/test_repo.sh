#!/usr/bin/env bash

source /opt/ros/foxy/setup.bash
source /root/sim_ws/install/setup.bash

xvfb-run -a ros2 launch lite3_sim spawn_lite3_kinematic.launch.py gui:=false > /tmp/sim.log 2>&1 &
sim_pid=$!

sleep 8

ros2 launch lite3_sim repo_bridge.launch.py > /tmp/repo.log 2>&1 &
repo_pid=$!

sleep 4

echo "===== nodes ====="
ros2 node list --no-daemon

echo "===== repo topics ====="
ros2 topic list | grep -E 'inspection|arm|cmd_vel'

echo "===== start inspection ====="
ros2 service call /inspection/start std_srvs/srv/Trigger "{}" 2>&1 | tail -8

echo "===== cmd_vel guard test ====="
timeout 2 ros2 topic pub --rate 10 /inspection/cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
echo "cmd_vel sample:"
timeout 2 ros2 topic echo /cmd_vel 2>&1 | head -12
echo "odom sample:"
timeout 2 ros2 topic echo /odom 2>&1 | sed -n '1,18p'

echo "===== fake detection ====="
for i in 1 2 3 4 5 6; do
  ros2 topic pub --once /inspection/detections std_msgs/msg/String "{data: '{\"detections\": [{\"class_name\": \"block\", \"confidence\": 0.9}]}'}" >/dev/null 2>&1 || true
  sleep 0.1
done
sleep 1
echo "events:"
timeout 2 ros2 topic echo /inspection/event 2>&1 | head -8

echo "===== confirm pick ====="
ros2 service call /inspection/confirm_pick std_srvs/srv/Trigger "{}" 2>&1 | tail -8
sleep 1
echo "arm status:"
timeout 2 ros2 topic echo /arm/status 2>&1 | head -10

kill "$repo_pid" 2>/dev/null || true
kill "$sim_pid" 2>/dev/null || true
wait "$repo_pid" 2>/dev/null || true
wait "$sim_pid" 2>/dev/null || true

echo "===== repo log errors ====="
grep -iE 'error|exception|failed' /tmp/repo.log | tail -20 || true
