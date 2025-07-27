# Dockerfile

FROM osrf/ros:humble-desktop-full

ARG COLCON_WORKSPACE

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN apt update && apt upgrade -y

# Install essential packages
RUN apt install -y wget python3-rosdep

# Init and update rosdep
RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init && \
    rosdep update --rosdistro=${ROS_DISTRO}

# Install ROS 2 control
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster

# Install nav2
RUN apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install Gazebo Classic and ROS 2 Gazebo integration
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-dev-tools \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-dev

# Install additional ROS2 utils
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-rqt-gui-py \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-plotjuggler-ros

# Install adafruit i2c librarries
RUN apt install -y \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-smbus \
    i2c-tools

RUN pip3 install --no-cache-dir \
    RPi.GPIO \
    adafruit-blinka

# Set up the workspace directory
WORKDIR $COLCON_WORKSPACE/src

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"     >> ~/.bashrc

