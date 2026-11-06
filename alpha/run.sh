#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ${2:-} == --scout ]]; then
  [[ $# == 2 ]] || { echo 'usage: run.sh seed --scout' >&2; exit 2; }
  exec python3 collect.py "$1"
fi
exec python3 mission.py "$@"
