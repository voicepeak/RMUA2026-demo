#!/usr/bin/env bash
# 启动 YOLO Gate 检测节点（xal 环境；torch cu128）
# 可覆盖环境变量: MODEL / ROUTE / IMGSZ / CONF，或追加任意 ROS 私有参数
set -e

CONT=rmua_noetic
MODEL=${MODEL:-/workspace/repo/yolo/weights/best.pt}
ROUTE=${ROUTE:-/workspace/rmua_ws/src/route_follower/config/route_1_3.yaml}
IMGSZ=${IMGSZ:-960}
CONF=${CONF:-0.35}

docker start "$CONT" >/dev/null 2>&1 || true

# 先清理旧 YOLO 实例, 避免多个节点同时发布 gate_map
docker exec "$CONT" bash -lc 'pkill -9 -f "[g]ate_yolo_node"' >/dev/null 2>&1 || true
sleep 1

if [ -t 0 ] && [ -t 1 ]; then TTY="-it"; else TTY="-i"; fi

exec docker exec $TTY "$CONT" bash -c "
  source /opt/ros/noetic/setup.bash
  source /workspace/rmua_ws/devel/setup.bash
  export PYTHONPATH=\$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
  export YOLO_CONFIG_DIR=/tmp/Ultralytics
  exec /opt/conda/envs/xal/bin/python \
    /workspace/rmua_ws/src/rmua_gate_vision/scripts/gate_yolo_node.py \
    _model:=$MODEL _route_file:=$ROUTE _imgsz:=$IMGSZ _conf:=$CONF $*
"
