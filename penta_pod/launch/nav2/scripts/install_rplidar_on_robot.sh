#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$SCRIPT_DIR"

# Extract workspace root directory, by splitting on first src directory
WORKSPACE_ROOT=$(awk -F'src' '{print $1}' <<< "$SCRIPT_DIR")

ROS_DISTRO=humble

# install vcs tools
sudo apt install -y python3-vcstool
sudo apt install -y python3-colcon-common-extensions

# clone nav2 repos
vcs import ${WORKSPACE_ROOT}/src < ./penta_nav_repos.yaml

# Build the workspace
cd $WORKSPACE_ROOT
colcon build --packages-select rplidar_ros
colcon build --packages-up-to penta_pod --parallel-workers 1