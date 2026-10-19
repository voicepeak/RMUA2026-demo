#!/usr/bin/env bash
set -euo pipefail
CONT=${RMUA_CONTAINER:-rmua_noetic}
docker inspect "$CONT" >/dev/null
docker start "$CONT" >/dev/null
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
project=$(basename "$root")
parent=$(basename "$(dirname "$root")")
if [[ "$project" != main || "$parent" != RUMA-by-helinjun ]]; then
  echo 'Update the /workspace/main mapping when relocating this workspace.' >&2
  exit 1
fi
tty_args=()
if [[ -t 0 && -t 1 ]]; then tty_args=(-it); fi
if [[ $# == 0 ]]; then set -- bash; fi
exec docker exec "${tty_args[@]}" -u "$(id -u):$(id -g)" -w /workspace/main "$CONT" bash -c '
  source /opt/ros/noetic/setup.bash
  # Reuse installed message definitions without loading legacy flight nodes.
  source /workspace/rmua_ws/devel/setup.bash
  if [[ -f /workspace/main/.runtime_ws/devel/setup.bash ]]; then
    source /workspace/main/.runtime_ws/devel/setup.bash
  fi
  export ROS_HOME=/tmp/rmua_next_ros
  mkdir -p "$ROS_HOME"
  export PYTHONPATH="/workspace/main:${PYTHONPATH:-}"
  exec "$@"
' rmua-next-env "$@"
