#!/usr/bin/env bash
set -euo pipefail

sg docker -c "docker exec -it lite3_sim bash -lc 'source /opt/ros/foxy/setup.bash && source /root/sim_ws/install/setup.bash && exec bash'"
