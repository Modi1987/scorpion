# pentapod IMU unit

IMU mpu6050 attached to esp32 that is connected to raspberry-pi over usb

Contents are:

- arduino: the arduino project for esp32

- pentapod_imu: the package that reads serial data from esp32 and publishes over ROS2


# Hardware

MPU6050 is connected to ESP32 over i2c and ESP32 is connected to robot over USB

# How to launch


Launch the imu node:

```
ros2 launch pentapod_imu pentapod_imu.launch.py
```

# Visualization

To visualize in Rviz2 wih imu tools, install imu tools

```
sudo apt-get update
sudo apt-get install ros-${ROS_DISTOR}-imu-tools
```

Run the transform from IMU link to base_link

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 -3.14159265358 base_link imu_link
```


