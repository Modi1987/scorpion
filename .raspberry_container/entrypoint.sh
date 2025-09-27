# .deploycontainer/entrypoint.sh

#!/bin/bash
set -e

# Source the ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Navigate to the workspace directory
cd /root/ros_ws

# Install dependencies and build the workspace
# Skip even in case of errors
rosdep install --from-paths src --ignore-src -r -y  || true

# Colcon build
#colcon build --packages-up-to penta_pod --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --parallel-workers 2

# Launch the application
#source install/setup.bash
#ros2 launch penta_pod ros2_control_penta.launch.py &

touch /root/ros_ws/.ready

# Keep the container running
tail -f /dev/null

