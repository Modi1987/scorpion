# Dockerfile

FROM osrf/ros:jazzy-desktop-full

ARG COLCON_WORKSPACE
ARG ROS_DISTRO=jazzy

SHELL [ "/bin/bash" , "-c" ]

# Upgrade all packages
RUN apt update && apt upgrade -y

# Install python packages
RUN apt install -y wget python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-runner \
    python3-setuptools \
    python3-pip

# Install important tools
RUN apt install -y \
    nano \
    curl \
    htop \
    git \
    sudo \
    lsb-release

# Install NVIDIA runtime dependencies
RUN apt install -y nvidia-cuda-toolkit

# Install ROS 2 control and additional ROS2 utilities
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-ros2-control-cmake \
    ros-${ROS_DISTRO}-forward-command-controller \
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
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}--plotjuggler-ros

# gazebo ros2 control packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-gz-ros2-control

# Set up the workspace directory
WORKDIR $COLCON_WORKSPACE/src

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"     >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=91"                      >> ~/.bashrc
