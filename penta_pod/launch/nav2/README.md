# About

Setting up autonomous navigation for the penta_pod robot,


Hardware requirement:

- pentapod robot connected to wifi

- external pc connected to same netwrok as robot

- rplidar A1, mounted on the robot and connected to raspberry pi of the pentapod robot


Software requirement, you have to perforn sinple installation steps both on the robot and the PC as in the following:


# Software installion on the robot

To install navigation stack NAV2 on PC, from inside the folder [scripts](./scripts/) run the installation script:

```
./install_rplidar_on_robot.sh
```

# Software installion on PC

To install navigation stack NAV2 on PC, from inside the folder [scripts](./scripts/) run the installation script:

```
./install_nav2_on_pc.sh
```

# What to launch on the robot

Turn on the robot hardware using:

```
ros2 launch penta_pod realhardware_bringup.launch.py
```


# What to launch on an external PC


To run the navigation and visualize with Rviz, On an external computer running NAV2 run the following:

```
ros2 launch penta_pod nav2_bringup.launch.py
```


# Extra considerations

Make sure of the following

- your PC and the robot are connected on same wifi

- you are using cyclone dds both on the robot and on the external pc

- you have the same `ROS_DOAMIN_ID` on both the PC and the raspberry-pi, to do so do the following both on raspberry-pi and on your PC

```
nano ~/.bashrc
```

then add the following at the end of the file

```
export ROS_DOAMIN_ID=38
```

