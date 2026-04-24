#!/bin/bash
docker run --rm -it \
  --net=host \
  --ipc=host \
  --name ros \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/root/tfg_panda_ws \
  tfg-panda:humble