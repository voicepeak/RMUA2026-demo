#!/usr/bin/env bash
# 单入口：./run.sh <seed> [--race|--collect]
# 依赖：Docker 容器（main 的 rmua 镜像）已可用，见 deployment/README.md
set -euo pipefail
cd "$(dirname "$0")"

SEED=${1:?usage: run.sh <seed> [--race|--collect]}
MODE=${2:---race}

exec python3 mission.py --seed "$SEED" --mode "${MODE#--}" "$@"
