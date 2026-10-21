#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:-$HOME/lite3_industrial_inspection_ws}"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
  echo "ERROR: /opt/ros/foxy/setup.bash not found. Switch the Lite3 image to ROS 2 first."
  exit 1
fi

source /opt/ros/foxy/setup.bash
cd "$WORKSPACE"

python3 - <<'PY'
required = ["rclpy", "cv2", "numpy", "cv_bridge"]
optional = ["serial"]
for module in required + optional:
    try:
        __import__(module)
        print("OK  ", module)
    except Exception as exc:
        prefix = "WARN" if module in optional else "FAIL"
        print(prefix, module, exc)
PY

colcon build --symlink-install
echo "source $WORKSPACE/install/setup.bash" >> "$HOME/.bashrc"

echo
echo "Build complete. Run:"
echo "  source $WORKSPACE/install/setup.bash"
echo "  ros2 launch lite3_inspection inspection.launch.py"
