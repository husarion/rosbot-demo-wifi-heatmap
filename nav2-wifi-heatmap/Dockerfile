# choose ROS distribudion based on build argument
FROM husarion/nav2:galactic

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . .

RUN apt update && apt install -y \
        python3-pip \
        python3-colcon-common-extensions && \
    apt upgrade -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install && \
    # make the image smaller
    apt-get remove -y --purge \
        python3-pip \
        python3-colcon-common-extensions && \
    apt-get autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros2_ws/ros_entrypoint.sh"]

CMD ["bash"]
