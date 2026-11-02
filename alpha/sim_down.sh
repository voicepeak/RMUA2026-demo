#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")" && pwd)
state="$root/.sim-owner"
[[ -f $state ]] || { echo 'No alpha-owned simulator' >&2; exit 1; }
read -r cont < "$state"
[[ $cont == "${RMUA_CONTAINER:-rmua_next_gpu}" ]] || { echo 'Container ownership mismatch' >&2; exit 1; }
docker stop -t 15 "$cont" >/dev/null
[[ $(docker inspect -f '{{.State.Running}}' "$cont") == false ]] || exit 1
# A stopped container has no live processes, including simulator and master.
[[ -z $(docker top "$cont" 2>/dev/null || true) ]] || { echo 'Container processes remain' >&2; exit 1; }
rm "$state"
echo 'Simulator and ROS master stopped.'
