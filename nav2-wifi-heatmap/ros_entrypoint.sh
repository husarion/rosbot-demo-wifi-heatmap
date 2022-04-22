#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup Fast DDS RMW
source /fastdds_overlay/install/setup.bash

source "/ros2_ws/install/setup.bash"

exec "$@"
