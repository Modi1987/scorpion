# About

This is nav2 launcher file for the penta_pod robot,

To work, you ineed to install rplidar on the robot and connect it to USB port of the raspberry pi


# What to launch on the robot

Turn on the robot hardware using:

```
ros2 launch penta_pod realhardware_bringup.launch.py
```


Turn on rplidar on the robot:

```
ros2 launch rplidar_ros rplidar.launch.py
```

To install rplidar on the robot raspberry pi:

```
cd ~/your_workspace/src

git clone https://github.com/babakhani/rplidar_ros2.git

cd ~/your_workspace

colcon build
```

# What to launch on an external PC


On an external computer running NAV2 run the following:

```
ros2 launch penta_pod nav2_bringup.launch.py
```


# Notes

Make sure of the following

- your PC and the robot are connected on same wifi

- you are using cyclone dds on the robot and on the external pc

- you have the same `ROS_DOAMIN_ID` on both the PC and the raspberry-pi, to do so do the following both on raspberry-pi and on your PC

```
nano ~/.bashrc
```

then add the following at the end of the file

```
export ROS_DOAMIN_ID=38
```

