#!/usr/bin/env bash
# One fresh, read-only P0 scene; no controller is started.
set -euo pipefail
CONT=${RMUA_CONTAINER:-rmua_noetic}
SEED=${1:-123}
MODE=${2:-offscreen}
[[ "$SEED" =~ ^[0-9]+$ ]] || { echo 'Seed must be a nonnegative integer.' >&2; exit 2; }
[[ "$MODE" == offscreen || "$MODE" == render ]] || { echo 'Mode: offscreen|render' >&2; exit 2; }
exec 9>/tmp/rmua-next-simulator.lock
flock -n 9 || { echo 'Another main simulator is active.' >&2; exit 1; }
docker start "$CONT" >/dev/null
if docker exec "$CONT" pgrep -f '[R]MUA-Linux|[r]osmaster|[r]oute_follower.py' >/dev/null; then
  echo 'Close the existing simulator/ROS/controller before starting a new scene.' >&2
  exit 1
fi
exec docker exec -u 1000:1000 -e DISPLAY="${DISPLAY:-:0}" \
  -w /workspace/main/simulator/simulator_12.0.0.5 "$CONT" \
  bash -c '
    set -e
    source /opt/ros/noetic/setup.bash
    export ROS_HOME=/tmp/rmua_next_ros
    mkdir -p "$ROS_HOME"
    setsid roscore >"$ROS_HOME/roscore.log" 2>&1 &
    ros_pid=$!
    echo "$ros_pid" >"$ROS_HOME/ros.pid"
    sim_pid=
    cleanup() {
      if [[ -n "$sim_pid" ]]; then kill -- "-$sim_pid" 2>/dev/null || true; fi
      kill -- "-$ros_pid" 2>/dev/null || true
      rm -f "$ROS_HOME/sim.pid" "$ROS_HOME/ros.pid"
    }
    trap cleanup EXIT
    trap "exit 130" INT TERM
    ready=false
    for attempt in {1..100}; do
      if rosparam list >/dev/null 2>&1; then ready=true; break; fi
      sleep .1
    done
    "$ready" || { echo "ROS master failed to start" >&2; exit 1; }
    if [[ "$2" == offscreen ]]; then flags=(-RenderOffscreen); else flags=(-windowed -ResX=1280 -ResY=720); fi
    setsid ./Build/LinuxNoEditor/RMUA.sh seed "$1" "${flags[@]}" -settings=/workspace/main/configs/simulator_settings.json &
    sim_pid=$!
    echo "$sim_pid" >"$ROS_HOME/sim.pid"
    wait "$sim_pid"
  ' rmua-next-simulator "$SEED" "$MODE"
