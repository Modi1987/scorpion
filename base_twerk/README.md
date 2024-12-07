# base_twerk

A package that makes the robot twerk

## Launch

Launch the action server

```
ros2 launch base_twerk base_twerk_action_server.launch.py
```

Launch the null space pose publisher

```
ros2 launch base_twerk null_pose_publisher.launch.py
```

## Action call

```
ros2 action send_goal /base_twerk_action base_twerk_msgs/action/BaseTwerkAction "rx: 0.05
ry: 0.05
rz: 0.0
phi_x: 1.7
phi_y: 0.0
phi_z: 0.0
w: 0.5
dance_time_millis: 10000.0"
```