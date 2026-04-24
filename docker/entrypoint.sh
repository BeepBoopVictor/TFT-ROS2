#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /root/tfg_panda_ws/install/setup.bash ]; then
  source /root/tfg_panda_ws/install/setup.bash
fi

exec "$@"