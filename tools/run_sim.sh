#!/usr/bin/env bash
# One owned scene; the harness starts control separately. Probes are diagnostic.
set -euo pipefail
CONT=${RMUA_CONTAINER:-rmua_noetic}
SEED=${1:-123}
MODE=${2:-offscreen}
DIAGNOSTIC=${3:-}
[[ "$SEED" =~ ^[0-9]+$ ]] || { echo 'Seed must be a nonnegative integer.' >&2; exit 2; }
[[ "$MODE" == offscreen || "$MODE" == render ]] || { echo 'Mode: offscreen|render' >&2; exit 2; }
[[ -z "$DIAGNOSTIC" || "$DIAGNOSTIC" == diagnostic-bridge-probe ]] || { echo 'Unknown diagnostic mode.' >&2; exit 2; }
probe_environment=(-e RMUA_BRIDGE_PROBE_ENABLED=0)
if [[ "$DIAGNOSTIC" == diagnostic-bridge-probe ]]; then
  [[ "${RMUA_BRIDGE_PROBE_MODE:-}" == observe || "${RMUA_BRIDGE_PROBE_MODE:-}" == serialize || "${RMUA_BRIDGE_PROBE_MODE:-}" == lifetime ]] || exit 2
  [[ -n "${RMUA_BRIDGE_PROBE_SO:-}" && -n "${RMUA_BRIDGE_PROBE_OUTPUT:-}" ]] || exit 2
  probe_environment=(-e RMUA_BRIDGE_PROBE_ENABLED=1 -e "RMUA_BRIDGE_PROBE_SO=$RMUA_BRIDGE_PROBE_SO"
                     -e "RMUA_BRIDGE_PROBE_MODE=$RMUA_BRIDGE_PROBE_MODE"
                     -e "RMUA_BRIDGE_PROBE_OUTPUT=$RMUA_BRIDGE_PROBE_OUTPUT")
fi
exec 9>/tmp/rmua-next-simulator.lock
flock -n 9 || { echo 'Another main simulator is active.' >&2; exit 1; }
docker start "$CONT" >/dev/null
if docker exec "$CONT" pgrep -f '[R]MUA-Linux|[r]osmaster|[r]oute_follower.py' >/dev/null; then
  echo 'Close the existing simulator/ROS/controller before starting a new scene.' >&2
  exit 1
fi
exec docker exec -u 1000:1000 -e DISPLAY="${DISPLAY:-:0}" \
  "${probe_environment[@]}" \
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
    launcher=(./Build/LinuxNoEditor/RMUA.sh seed "$1" "${flags[@]}" -settings=/workspace/main/configs/simulator_settings.json)
    if [[ "${RMUA_BRIDGE_PROBE_ENABLED:-0}" == 1 ]]; then
      launcher=(env "LD_PRELOAD=$RMUA_BRIDGE_PROBE_SO"
        "RMUA_BRIDGE_PROBE_ORIGINAL=/workspace/main/simulator/simulator_12.0.0.5/Build/LinuxNoEditor/RMUA/Plugins/AirSimROS/Binaries/Linux/libairsimros.so"
        "RMUA_BRIDGE_PROBE_MODE=$RMUA_BRIDGE_PROBE_MODE"
        "RMUA_BRIDGE_PROBE_OUTPUT=$RMUA_BRIDGE_PROBE_OUTPUT" "${launcher[@]}")
    fi
    setsid "${launcher[@]}" &
    sim_pid=$!
    echo "$sim_pid" >"$ROS_HOME/sim.pid"
    wait "$sim_pid"
  ' rmua-next-simulator "$SEED" "$MODE"
