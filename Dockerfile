FROM ros:galactic-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY navigation2/nav2_msgs src/nav2_msgs
COPY navigation2/nav2_common src/nav2_common
##COPY for main package will be added for final version, currently operating on bind mount

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-galactic-rmw-fastrtps-cpp && \
    apt upgrade -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install
    # apt-get remove -y --purge \
    # python3-pip \
    # python3-colcon-common-extensions && \
    # apt-get autoremove -y && apt clean && \
    # rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-opencv
RUN pip install opencv-python

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]