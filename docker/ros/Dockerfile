# The turtlebot3_isaacsim image: TurtleBot3 and this package on top of the
# Isaac Sim + ROS 2 Humble base built from docker/isaacsim-ros2/Dockerfile.humble.
# scripts/build_images.sh builds both, in order.

ARG BASE_IMAGE=isaacsim61-humble:ngc
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    python3-argcomplete \
    build-essential git nano \
 && rm -rf /var/lib/apt/lists/*

# --- TurtleBot3: description, real-robot drivers, sim worlds ------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-description \
    ros-${ROS_DISTRO}-turtlebot3-bringup \
    ros-${ROS_DISTRO}-turtlebot3-navigation2 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-ld08-driver \
    ros-${ROS_DISTRO}-coin-d4-driver \
 && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}
RUN mkdir -p ${COLCON_WS}/src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f ${COLCON_WS}/install/setup.bash ] && source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
