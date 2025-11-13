
# topperware_bot robot description package


Includes the urdf file and launch files for the topperware_bot

To test the urdf file, you can use the urdf_tutorial, install it if not laready installed on your system, then:


## To test the urdf alone:

```
ros2 launch urdf_tutorial display.launch.py model:="$(pwd)/penta.urdf.xacro"
```

The package also contains launch files that allows u to launch robot description on your urdf or run also rviz to visualize the penta_pod robot


```
ros2 launch penta_description penta_description_simple.launch.py

```

To run rviz:

```
ros2 launch penta_description penta_rviz.launch.py
```


## Move the joints in gazebo simulation


```
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{ 
  trajectory: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: ''
    },
    joint_names: ['limb0/joint0', 'limb0/joint2'],
    points: [
      {
        positions: [1.0, 0.5],
        velocities: [0.0, 0.0],
        time_from_start: {sec: 2, nanosec: 0}
      }
    ]
  },
  path_tolerance: [],
  goal_tolerance: [],
  goal_time_tolerance: {sec: 1, nanosec: 0}
}"
```
