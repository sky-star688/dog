#!/usr/bin/env bash
# Start Dog 1 (lite3_01) – LiDAR SLAM + depth camera + patrol.
# Launches ONLY Dog 1 so it can be debugged independently.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "ERROR: workspace is not built. Run: bash scripts/build.sh dog1" >&2
  exit 1
fi
if [ "$(id -u)" -eq 0 ]; then
  echo "ERROR: do not run as root." >&2
  exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

echo "============================================"
echo "  Dog 1 (lite3_01) – Mapping + Patrol"
echo "  Hardware: LiDAR + Depth Camera"
echo "  Map shared on: /shared_map"
echo "============================================"
echo ""
echo "Before starting Dog 1, ensure:"
echo "  - LiDAR driver is running"
echo "  - Depth camera is running (color + aligned depth)"
echo "  - slam_toolbox is running (publishes /map)"
echo "  - Nav2 is running"
echo ""

exec ros2 launch lite3_01_mapping mapping_dog.launch.py \
  robot_id:=lite3_01 \
  namespace:=/ \
  "$@"
