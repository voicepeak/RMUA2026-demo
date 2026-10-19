#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$root/.runtime_ws/src"
if [[ ! -e "$root/.runtime_ws/src/airsim_ros" ]]; then
  ln -s ../../interfaces/ros/airsim_ros "$root/.runtime_ws/src/airsim_ros"
fi
exec "$root/tools/enter_env.sh" catkin_make -C /workspace/main/.runtime_ws \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
