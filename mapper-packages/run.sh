#!/bin/bash

set -e

source "/opt/ros/galactic/setup.bash"
source "/ros2_ws/install/setup.bash"
echo "saving map..."
ros2 run nav2_map_server map_saver_cli -f /map/map --ros-args -p save_map_timeout:=100.0
sleep 3

ros2 launch mappers_bringup mappers.launch.py

exec "$@"