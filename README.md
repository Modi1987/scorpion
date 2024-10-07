# SCORPION

This is a ROS2 software for the following older project [youtube video available here.](https://youtu.be/kcvJR5mcb1o?si=lxt_06UO4189CPcX)


## How to Build the Package

The package is based on ROS2 Humble and can be compiled on a laptop or a Raspberry Pi. To compile:

Create your workspace folder

```
mkdir -p ~/my_ws/src
cd ~/my_ws/src
```

Clone the repo

```
git clone git@github.com:Modi1987/scorpion.git
```

Compile the workspace

```
cd ~/my_ws/src
colcon build
```


## How to run on real robot

Launch the following file:

```
ros2 launch penta_pod realhardware_bringup.launch.py
```

To teleoperate the robot from your PC, use the teleop_twist_keyboard:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Enjoy move the robot around

## How to run and visualize in rviz:

To run the package in rviz (and visualize the robot moving):

- first, run the joints_aggregator, which will aggregate the joitns angles published individually by each limb into one /joint_states message

```
ros2 launch joints_aggregator joints_aggregator.launch.py 
```

- second, run the rviz simuation

```
ros2 launch penta_pod penta_rviz.launch.py
```

- third, run the penta_pod core package, this subscripes on feet positions and publishes joints angles for each limb

```
ros2 launch penta_pod penta_pod.launch.py
```

- fourth, run the gait generator, which move the robot feet according to twist command

```
ros2 launch gait_generator gait_generator.launch.py
```

- fifth, you can stream the twist command /cmd_vel using teleop_twist_keyboard

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


# Other ways to move the simulation around

Invoke the ik solver by publishing a target foot position

```
ros2 topic pub /limb0/xyz_msg limb_msgs/msg/Pxyz "{"x": 0.1, "z": -0.1}"
```

Move feet up and down

```
ros2 run test_foot_pos test_gait_node
```

To animate the simulation you can stream feet positions

```
ros2 launch test_foot_pos test_foot_pos.launch.py
```
