#!/usr/bin/env bash
set -u
PIPE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
touch "$PIPE/STOP"
pkill -f "RL/learn.py" 2>/dev/null || true
pkill -f "power_guard.sh" 2>/dev/null || true
pkill -f "python pipeline.py" 2>/dev/null || true
pkill -f "loop_supervisor.py" 2>/dev/null || true
echo "stopped"
