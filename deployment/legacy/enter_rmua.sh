#!/usr/bin/env bash
# 进入 RMUA Noetic 开发容器（自动启动/重建容器，并 source ROS 与工作空间）
set -e

CONT=rmua_noetic
REPO_DIR=/home/tianbot/RUMA-by-helinjun
IMAGE=rmua/noetic-xal:latest

xhost +local:docker >/dev/null 2>&1 || true

if ! docker inspect "$CONT" >/dev/null 2>&1; then
  docker image inspect "$IMAGE" >/dev/null 2>&1 || IMAGE=osrf/ros:noetic-desktop-full
  docker run -dit --name "$CONT" \
    --gpus all --network host --ipc host \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$REPO_DIR":/workspace \
    -w /workspace/rmua_ws \
    "$IMAGE" bash >/dev/null
else
  docker start "$CONT" >/dev/null 2>&1 || true
fi

docker exec $( [ -t 0 ] && [ -t 1 ] && echo -it || echo -i ) "$CONT" bash -c '
  source /opt/ros/noetic/setup.bash
  source /workspace/rmua_ws/devel/setup.bash
  exec bash
'
