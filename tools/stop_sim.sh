#!/usr/bin/env bash
# Stop only process groups recorded by main/tools/run_sim.sh.
set -euo pipefail
CONT=${RMUA_CONTAINER:-rmua_noetic}
exec docker exec -u 1000:1000 "$CONT" bash -c '
  for name in sim ros; do
    path=/tmp/rmua_next_ros/$name.pid
    [[ -f "$path" ]] || continue
    read -r pid <"$path"
    [[ "$pid" =~ ^[0-9]+$ && "$pid" -gt 1 ]] || exit 1
    [[ -e /proc/$pid/cmdline ]] || continue
    command=$(tr "\0" " " </proc/$pid/cmdline)
    if [[ "$name" == sim && "$command" != *RMUA* ]] || [[ "$name" == ros && "$command" != *roscore* ]]; then
      echo "Recorded PID changed; refusing to stop $pid" >&2
      exit 1
    fi
    kill -- "-$pid" 2>/dev/null || true
    stopped=false
    for attempt in {1..100}; do
      if ! kill -0 -- "-$pid" 2>/dev/null; then stopped=true; break; fi
      sleep .1
    done
    "$stopped" || { echo "Process group $pid did not stop in 10s" >&2; exit 1; }
  done
'
