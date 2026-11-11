#!/usr/bin/env bash
# Start Dog 2 (lite3_02) – OpenMV + depth camera + MyArm-2 picking.
# Launches ONLY Dog 2 so it can be debugged independently.
# Dog 2 waits for Dog 1's frozen /shared_map and AMCL convergence.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "ERROR: workspace is not built. Run: bash scripts/build.sh dog2" >&2
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
echo "  Dog 2 (lite3_02) – Picking + Arm"
echo "  Hardware: OpenMV + Depth Camera + MyArm-2"
echo "  Waits for: /shared_map (frozen+verified)"
echo "============================================"
echo ""
echo "Before starting Dog 2, ensure:"
echo "  - Dog 1 is running and its map is FROZEN:"
echo "      ros2 service call /dog1/freeze_map std_srvs/srv/Trigger '{}'"
echo "  - initial_x/y/yaw in config/picking.yaml are SURVEYED"
echo "  - MyArm-2 is powered and connected (dry_run: true first)"
echo "  - OpenMV is connected (or set enable_openmv:=false)"
echo "  - Nav2 + AMCL are running"
echo ""

exec ros2 launch lite3_02_picking picking_dog.launch.py \
  robot_id:=lite3_02 \
  namespace:=/ \
  "$@"
