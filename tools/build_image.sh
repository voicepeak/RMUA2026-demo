#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
image=${1:-rmua-next:p1-development}
revision=$(git -C "$root" rev-parse HEAD)
if [[ -n $(git -C "$root" status --porcelain) ]]; then revision="$revision-dirty"; fi
exec docker build --network=none --build-arg "GIT_REVISION=$revision" \
  -f "$root/deployment/Dockerfile" -t "$image" "$root"
