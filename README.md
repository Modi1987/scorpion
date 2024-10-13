# Scorpion

This is a ROS2 software for the following older project [youtube video available here.](https://youtu.be/kcvJR5mcb1o?si=lxt_06UO4189CPcX)


## How to Build the Package

The package is based on ROS2 Humble (on Ubuntu 22.04) and can be compiled on a laptop or a Raspberry Pi. To compile:

Create your workspace folder

```
mkdir -p ~/my_ws/src
cd ~/my_ws/src
```

Clone the repo

```
git clone git@github.com:Modi1987/scorpion.git
```

There are bash scripts that will allow you to build your workspace you can do this on your PC if you want to run RVIZ simulations, or on Raspberry-pi4 if you want to run on real-robot

```
cd ~/my_ws/src/scorpion/scripts
./setup_penta_workspace.sh
```

## Setting up on real robot Raspberry pi4

besides to the previous steps to setup your workspace, to control the real-robot you will need to configure the i2c bus on Raspberry-pi 4, to do so on the Raspberry-pi 4:

```
cd ~/my_ws/src/scorpion/scripts
./setup_i2c_on_raspberry_pi4.sh
```

To bring-up the robot automatically each time you boot the robot, run the following script on the Raspberry Pi

```
cd ~/my_ws/src/scorpion/scripts
./setup_penta_bringup_on_boot.sh
```

Then reboot the Raspberry Pi

Finally, you can move the robot around from external PC using keyboard

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Run in simulation

You can run the simulation using the command

```
ros2 launch penta_pod penta_simn_rviz.launch.py
```

You can move the robot around using

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
