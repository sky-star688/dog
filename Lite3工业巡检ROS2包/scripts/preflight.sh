#!/usr/bin/env bash
# Read-only preflight: verify external topics/actions exist before starting.
set -euo pipefail

fail=0
check() {
  local label="$1"; shift
  if "$@" >/dev/null 2>&1; then
    echo "OK   $label"
  else
    echo "FAIL $label"
    fail=1
  fi
}

check "action navigate_to_pose"    ros2 action list
check "topic /leg_odom2"           ros2 topic list
check "topic /camera/color/image_raw" ros2 topic list
check "topic /camera/aligned_depth_to_color/image_raw" ros2 topic list

if ros2 topic list | grep -qx "/cmd_vel"; then
  echo "WARN /cmd_vel exists before guard start; verify publisher ownership"
fi
if ros2 node list | grep -qx "/cmd_vel_guard"; then
  echo "OK   guard is running"
else
  echo "WARN guard is not running yet"
fi

exit "$fail"
