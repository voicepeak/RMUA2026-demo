#!/usr/bin/env bash
# 从空地图启动；先运行 ./run_yolo.sh 提供本轮在线观测。
# 历史静态回放仅显式传入 gates_file:=... 时启用。
set -e

CONT=rmua_noetic
docker start "$CONT" >/dev/null 2>&1 || true

# 先清理旧控制器, 避免多实例抢发 vel_body_cmd
docker exec "$CONT" bash -lc 'pkill -9 -f "[r]oute_follower.py"; pkill -9 -f "[r]oslaunch route_follower"' >/dev/null 2>&1 || true
sleep 1

if [ -t 0 ] && [ -t 1 ]; then TTY="-it"; else TTY="-i"; fi

exec docker exec $TTY "$CONT" bash -c "
  source /opt/ros/noetic/setup.bash
  source /workspace/rmua_ws/devel/setup.bash
  exec roslaunch route_follower route_follower.launch \
    gates_file:= guides_file:= wait_for_first_map:=true \
    gate_z_uses_offset:=false \
    cruise_speed:=10.0 normal_speed_floor:=4.0 \
    yaw_control:=true yaw_lookahead:=25.0 k_yaw:=1.2 yaw_accel_limit:=2.0 \
    $*
"
