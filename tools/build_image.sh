#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
image=${1:-rmua-next:p1-development}
revision=$(git -C "$root" rev-parse HEAD)
if [[ -n $(git -C "$root" status --porcelain) ]]; then revision="$revision-dirty"; fi
docker build --network=none --build-arg "GIT_REVISION=$revision" \
  -f "$root/deployment/Dockerfile" -t "$image" "$root"
size=$(docker image inspect "$image" --format '{{.Size}}')
((size < 15000000000)) || { echo 'Development image exceeds the official size limit.' >&2; exit 2; }
# A successful docker build alone does not exercise the packaged Python modules.
exec docker run --rm --network none "$image" \
  python3 -m unittest discover -s tests/unit -q
