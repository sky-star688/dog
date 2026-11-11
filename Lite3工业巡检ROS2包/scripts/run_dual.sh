#!/usr/bin/env bash
# Start the full dual-dog collaborative system.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [ ! -f "$ROOT/install/setup.bash" ]; then
  echo "ERROR: workspace is not built. Run: bash scripts/build.sh" >&2
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
echo "  Dual-Dog Collaborative System"
echo ""
echo "  Dog 1 (lite3_01): LiDAR SLAM + Patrol"
echo "  Dog 2 (lite3_02): OpenMV + MyArm-2 Picking"
echo "  fleet_manager:    Coordination"
echo ""
echo "  Map: Dog 1 -> /shared_map -> Dog 2"
echo "============================================"

exec ros2 launch fleet_manager dual_dog.launch.py \
  inspector_id:=lite3_01 \
  operator_id:=lite3_02 \
  "$@"
