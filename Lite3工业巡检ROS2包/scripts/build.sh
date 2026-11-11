#!/usr/bin/env bash
# Build one or all packages in this workspace.
# Usage:
#   bash scripts/build.sh          # all packages
#   bash scripts/build.sh dog1     # lite3_inspection + lite3_01_mapping
#   bash scripts/build.sh dog2     # lite3_inspection + lite3_02_picking
#   bash scripts/build.sh fleet    # fleet_interfaces + fleet_manager
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DISTRO="${ROS_DISTRO:-foxy}"

if [ "$(id -u)" -eq 0 ]; then
  echo "ERROR: do not build as root; source the workspace as the robot user." >&2
  exit 1
fi
if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon is not installed." >&2
  exit 1
fi
if [ ! -f "/opt/ros/$DISTRO/setup.bash" ]; then
  echo "ERROR: /opt/ros/$DISTRO/setup.bash not found (ROS_DISTRO=$DISTRO)." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "/opt/ros/$DISTRO/setup.bash"

TARGET="${1:-all}"
case "$TARGET" in
  dog1)  PACKAGES="lite3_inspection lite3_01_mapping" ;;
  dog2)  PACKAGES="lite3_inspection lite3_02_picking" ;;
  fleet) PACKAGES="fleet_interfaces fleet_manager" ;;
  all|*) PACKAGES="fleet_interfaces lite3_inspection fleet_manager lite3_01_mapping lite3_02_picking" ;;
esac
echo "Building: $PACKAGES"

python3 - <<'PY'
required = ["rclpy", "cv2", "numpy", "cv_bridge", "yaml", "tf2_ros"]
optional = ["serial"]
failed = False
for module in required:
    try:
        __import__(module)
        print("OK  ", module)
    except Exception as exc:
        print("FAIL", module, exc)
        failed = True
for module in optional:
    try:
        __import__(module)
        print("OK  ", module)
    except Exception as exc:
        print("WARN", module, exc)
raise SystemExit(1 if failed else 0)
PY

cd "$ROOT"
# shellcheck disable=SC2086
colcon build \
  --base-paths "$ROOT/src" \
  --packages-select $PACKAGES \
  --symlink-install \
  --event-handlers console_direct+

echo
echo "Build finished. Source the workspace:"
echo "  source $ROOT/install/setup.bash"
