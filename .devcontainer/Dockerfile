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
    rosdep update --rosdistro=humble

# Set up the workspace directory
WORKDIR $COLCON_WORKSPACE/src

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"     >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=91"                      >> ~/.bashrc
