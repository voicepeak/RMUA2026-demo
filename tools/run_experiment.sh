#!/usr/bin/env bash
# Positional arguments preserve spaces and prevent shell evaluation of launch overrides.
set -euo pipefail
exec docker exec -i rmua_noetic bash -c '
  source /opt/ros/noetic/setup.bash
  source /workspace/rmua_ws/devel/setup.bash
  exec python3 /workspace/repo/tools/run_experiment.py "$@"
' bash "$@"
