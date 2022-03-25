#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source "/ros2_ws/install/setup.bash"

exec "$@"