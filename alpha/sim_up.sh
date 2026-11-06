#!/usr/bin/env bash
set -euo pipefail
seed=${1:?usage: sim_up.sh seed}
[[ $seed =~ ^[0-9]+$ ]] || exit 2
cont=${RMUA_CONTAINER:-rmua_next_gpu}
root=$(cd "$(dirname "$0")" && pwd)
state="$root/.sim-owner"
if [[ -n ${DISPLAY:-} ]]; then
  display=$DISPLAY
else
  sockets=(/tmp/.X11-unix/X[0-9]*)
  [[ ${#sockets[@]} == 1 && -S ${sockets[0]} ]] || { echo 'Set DISPLAY to the active X display' >&2; exit 1; }
  display=:${sockets[0]##*X}
fi
python3 -c 'import socket,sys; s=socket.socket(); s.settimeout(1); sys.exit(1 if s.connect_ex(("127.0.0.1",11311)) == 0 else 0)' || { echo 'ROS master port already in use' >&2; exit 1; }
[[ ! -e $state ]] || { echo 'Alpha scene already owned' >&2; exit 1; }
if ! docker inspect "$cont" >/dev/null 2>&1; then
  docker run -d --init --name "$cont" --network host --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e LD_LIBRARY_PATH= -e HOME=/tmp -v "$HOME/RUMA-by-helinjun:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix --entrypoint sleep "${RMUA_GPU_IMAGE:-rmua/noetic-xal:driver-recovery-20261012}" infinity >/dev/null
  docker stop "$cont" >/dev/null
fi
[[ $(docker inspect -f '{{.State.Running}}' "$cont") == false ]] || { echo 'Container already running; refusing to take ownership' >&2; exit 1; }
printf '%s\n' "$cont" > "$state"
docker start "$cont" >/dev/null
cleanup_failed_start() { "$root/sim_down.sh" || true; }
trap cleanup_failed_start EXIT
# The container is exclusively owned until sim_down; retain official launcher unchanged.
docker exec -d -u 1000:1000 -e DISPLAY="$display" -w /workspace/simulator/simulator_12.0.0.5 "$cont" bash -c 'source /workspace/rmua_ws/devel/setup.bash; exec ./run_simulator_offscreen.sh "$1" > "$2" 2>&1' alpha-sim "$seed" "${ALPHA_SIM_LOG:-/workspace/alpha/alpha/simulator.log}"
for ((i=0;i<120;i++)); do
  if docker exec "$cont" bash -c 'source /opt/ros/noetic/setup.bash; timeout 2 rostopic list 2>/dev/null | grep -Fxq /airsim_node/drone_1/front_left/Scene && timeout 2 rostopic list 2>/dev/null | grep -Fxq /airsim_node/drone_1/imu/imu'; then trap - EXIT; exit 0; fi
  if (( i >= 5 )) && ! docker exec "$cont" pgrep -f '[R]MUA-Linux-Shipping' >/dev/null; then
    echo 'Simulator exited; see simulator log'  >&2
    exit 1
  fi
  sleep 1
done
echo 'Simulator topics did not become ready' >&2
exit 1
