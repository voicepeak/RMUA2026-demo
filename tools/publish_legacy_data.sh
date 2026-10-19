#!/usr/bin/env bash
# Original archives stay read only; uploader dependencies live outside the ROS env.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python_bin=/home/tianbot/.local/share/rmua-hf-upload/venv/bin/python
[[ -x "$python_bin" ]] || { echo 'Upload environment is missing; see docs/dataset_publication.md' >&2; exit 1; }
cd "$root"
exec "$python_bin" -m evaluation.dataset_capture.publish_archive "$@"
