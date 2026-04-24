#!/bin/bash
source /opt/ros/humble/setup.bash
cd /root/tfg_panda_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install