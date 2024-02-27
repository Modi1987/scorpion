# SCORPION


git clone git@github.com:Modi1987/scorpion.git


## How to run and visualize in rviz:

To run the package in rviz and see the robot moving:


- first, run the penta_pod package, this subscripes on feed positions and publishes joints angles for each limb

```
ros2 launch penta_pod penta_pod.launch.py
```

- second, run the joints_aggregator, which will aggregate the joitns angles published individually by each limb into one /joint_states message

```
ros2 launch joints_aggregator joints_aggregator.launch.py 
```

- third, run the rviz simuation

```
ros2 launch penta_pod penta_rviz.launch.py
```

- fourth, invoke the ik solver by publishing a target foot position

```
ros2 topic pub /limb0/xyz_msg limb_msgs/msg/Pxyz "{"x": 0.1, "z": -0.1}"
```