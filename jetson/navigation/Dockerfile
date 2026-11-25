FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-dev libeigen3-dev python3-serial \
    ros-humble-eigen3-cmake-module ros-humble-rplidar-ros ros-humble-slam-toolbox \
    ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-navfn-planner \
    ros-humble-nav2-planner ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-controller ros-humble-nav2-regulated-pure-pursuit-controller \
    && rm -rf /var/lib/apt/lists/*

ARG ROBOT_UID=1000
ARG ROBOT_GID=1000
RUN groupadd -g "$ROBOT_GID" robot \
    && useradd -m -u "$ROBOT_UID" -g "$ROBOT_GID" -s /bin/bash robot
USER robot
ENV HOME=/home/robot
CMD ["sleep", "infinity"]
