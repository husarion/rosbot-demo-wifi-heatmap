#!/bin/bash

set -e

source "/opt/ros/galactic/setup.bash"
source "/ros2_ws/install/setup.bash"

ros2 run nav2_map_server map_saver_cli -f /map/map --ros-args -p save_map_timeout:=100.0

exec "$@"