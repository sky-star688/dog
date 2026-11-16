#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
WS="$ROOT/ws"
MODE="${1:-kinematic}"

if [ "$MODE" = "dynamic" ]; then
  LAUNCH="spawn_lite3.launch.py"
else
  LAUNCH="spawn_lite3_kinematic.launch.py"
fi

sg docker -c "docker rm -f lite3_sim >/dev/null 2>&1 || true"
sg docker -c "docker run --rm -i \
  --name lite3_sim \
  --net=host \
  -v ${WS}:/root/sim_ws \
  lite3_sim:foxy \
  bash -lc 'source /opt/ros/foxy/setup.bash && source /root/sim_ws/install/setup.bash && xvfb-run -a ros2 launch lite3_sim ${LAUNCH} gui:=false'"
