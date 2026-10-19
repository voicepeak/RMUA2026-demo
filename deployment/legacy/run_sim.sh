#!/usr/bin/env bash
# 启动官方 RMUA2026 模拟器（容器内运行；UE4 拒绝 root，故用 uid 1000）
# 用法: ./run_sim.sh [seed] [offscreen|render|background] [settings-path]
set -e

SEED=${1:-123}
MODE=${2:-offscreen}
SETTINGS=${3:-}
CONT=rmua_noetic
SIM_DIR=/workspace/simulator/simulator_12.0.0.5

docker start "$CONT" >/dev/null 2>&1 || true

# 旧实例会抢占 ROS master/GPU, 先清理, 避免叠加多个模拟器
if docker exec "$CONT" pgrep -f "RMUA-Linux|rosmaster" >/dev/null 2>&1; then
  echo "[run_sim] 发现旧模拟器或 ROS 实例, 先清理..."
  docker exec "$CONT" bash -lc 'pkill -9 -f "RMUA-Linux"; pkill -9 -f "RMUA.sh"; pkill -9 -f rosmaster; pkill -9 -f rosout' >/dev/null 2>&1 || true
  sleep 3
  docker restart "$CONT" >/dev/null 2>&1 || true
  sleep 2
fi

if [ "$MODE" = "background" ]; then
  if ! docker exec "$CONT" pgrep -f '^Xvfb :2' >/dev/null; then
    # Docker restart kills Xvfb without removing its shared X socket.
    docker exec -d "$CONT" bash -c 'rm -f /tmp/.X2-lock /tmp/.X11-unix/X2; exec Xvfb :2 -screen 0 1280x720x24 -ac -nolisten tcp > /tmp/rmua_xvfb.log 2>&1'
  fi
  for attempt in {1..30}; do
    if DISPLAY=:2 xwininfo -root >/dev/null 2>&1; then break; fi
    sleep .1
  done
  DISPLAY=:2 xwininfo -root >/dev/null
  DISPLAY=:2
  SCRIPT=run_simulator.sh
elif [ "$MODE" = "render" ]; then
  xhost +local:docker >/dev/null 2>&1 || true
  SCRIPT=run_simulator.sh
else
  SCRIPT=run_simulator_offscreen.sh
fi

exec docker exec -u 1000:1000 -e DISPLAY="${DISPLAY:-:0}" -w "$SIM_DIR" "$CONT" \
  bash -c 'export HOME=/tmp/rmua_user; mkdir -p "$HOME"; script=$1; seed=$2; settings=$3; if [ -n "$settings" ]; then exec "./$script" "$seed" -settings="$settings"; else exec "./$script" "$seed"; fi' \
  simulator-start "$SCRIPT" "$SEED" "$SETTINGS"
