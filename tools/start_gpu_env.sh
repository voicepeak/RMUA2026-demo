#!/usr/bin/env bash
# Create a separate development container using the host NVIDIA runtime.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(dirname "$root")
[[ "$(basename "$root")" == main && "$(basename "$workspace")" == RUMA-by-helinjun ]] || {
  echo 'Expected RUMA-by-helinjun/main workspace mapping.' >&2; exit 1;
}
container=${RMUA_CONTAINER:-rmua_next_gpu}
image=${RMUA_GPU_IMAGE:-rmua/noetic-xal:driver-recovery-20261012}
if docker inspect "$container" >/dev/null 2>&1; then
  echo "Container $container already exists; inspect it before reuse or removal." >&2
  exit 1
fi
docker run -d --init --name "$container" --network host --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e LD_LIBRARY_PATH= -e HOME=/tmp \
  -v "$workspace:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix \
  --entrypoint sleep "$image" infinity >/dev/null
echo "Started $container. Set RMUA_CONTAINER=$container for experiment commands."
