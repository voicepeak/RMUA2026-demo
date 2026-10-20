#!/usr/bin/env bash
# Diagnostics only. Build with the simulator container's C++/glibc ABI.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source_hash=$(sha256sum "$root/tools/diagnostics/bridge_probe.cpp" | cut -d' ' -f1)
output="$root/runs/probe_builds/$source_hash"
mkdir -p "$output"
probe_build_dir=$(mktemp -d "$output/.build.XXXXXX")
trap 'rm -rf -- "$probe_build_dir"' EXIT
cp "$root/tools/diagnostics/bridge_probe.cpp" "$probe_build_dir/bridge_probe.cpp"
[[ $(sha256sum "$probe_build_dir/bridge_probe.cpp" | cut -d' ' -f1) == "$source_hash" ]] || {
  echo 'Probe source changed during snapshot.' >&2; exit 2;
}
container_build_dir="/workspace/main/${probe_build_dir#"$root/"}"
bash "$root/tools/enter_env.sh" g++ -std=c++14 -O2 -fPIC -shared -Wall -Wextra -Werror \
  "$container_build_dir/bridge_probe.cpp" -pthread -ldl -o "$container_build_dir/bridge_probe.so"
# Atomic replacement leaves any already-loaded inode intact. Preserve the exact
# source copy as well as its hash; never rebuild into a running library's inode.
mv -f -- "$probe_build_dir/bridge_probe.so" "$output/bridge_probe.so"
mv -f -- "$probe_build_dir/bridge_probe.cpp" "$output/bridge_probe_source.cpp"
echo "$output/bridge_probe.so"
