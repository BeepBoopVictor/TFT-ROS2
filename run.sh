#!/bin/bash
set -e

CONTAINER_NAME="${CONTAINER_NAME:-tfg_panda_ws_dev}"

xhost +local:docker >/dev/null 2>&1 || true

docker run --rm -it \
  --net=host \
  --ipc=host \
  --name "${CONTAINER_NAME}" \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/root/tfg_panda_ws \
  --gpus all \
  tfg-panda:humble
