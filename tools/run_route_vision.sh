#!/usr/bin/env bash
# 在线地图模式（跑全赛道用）：先启动 YOLO（./run_yolo.sh），再跑本脚本。
# 连续门洞轨迹与持久高度地图；速度和安全上限统一由 launch 配置。
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
    use_gate_map:=true gate_map_min_support:=2 sigma_z_max:=0.6 \
    snap_gate_to_route:=false cruise_speed:=8.0 normal_speed_floor:=4.0 \
    k_ff_z:=1.0 map_commit_stable_frames:=1 \
    $*
"
