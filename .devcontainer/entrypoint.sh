# .deploycontainer/entrypoint.sh

#!/bin/bash
set -e

# Source the ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Navigate to the workspace directory
cd /root/ros_ws

# VSC import
vcs import src < /root/ros_ws/src/polypod/.devcontainer/vcs_repos.yaml || echo "ERROR: vcs import at entrypoint.sh"

# Install dependencies and build the workspace
rosdep install --from-paths src --ignore-src -r -y || echo "ERROR: rosdep install at entrypoint.sh"
# colcon build

# Keep the container running
tail -f /dev/null

