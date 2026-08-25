# About


MuJoCo simulation of the pentapod, to be used for RL later


First, clone mujoco repos

```bash
git clone https://github.com/ros-controls/mujoco_ros2_control.git

git clone https://github.com/pal-robotics/mujoco_vendor.git
```

Afterwards, install deps and then run `colcon build` on the workspace as in the following

```bash
rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y

colcon build
```


You can test the installation using

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py
```

## Install virtual environment in the container

```
apt-get update && apt-get install -y python3-venv
mkdir -p /root/.ros/ros2_control
python3 -m venv --system-site-packages /root/.ros/ros2_control/.venv
```


## How to launch

```
ros2 launch penta_mujoco_sim penta_mujoco.launch.py
```


## How it works

```
gait / joints_aggregator
        │  sensor_msgs/JointState
        ▼
  /joint_setpoints  ──►  robot_state_publisher (via remap → /joint_states)  ──►  RViz TF
        │
        │  (bridge in package ros_to_ros2_control_command_bridge)
        ▼
gazebo_forward_joint_command_bridge_node
        │  std_msgs/Float64MultiArray
        ▼
  /forward_position_controller/commands
        │
        ▼
  controller_manager → forward_position_controller → MujocoSystemInterface → MuJoCo
```