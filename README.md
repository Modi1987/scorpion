[![Powered by Dev Containers](https://img.shields.io/badge/Powered%20by-Dev%20Containers-blue?logo=devcontainers&logoColor=white)](https://containers.dev/)


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

cd scorpion

git checkout i2c_actuators
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

## Main launch file to control the real-robot

```
cd ~/my_ws

source install/setup.bash

ros2 launch penta_pod realhardware_bringup.launch.py
```

The robot shall move legs when the previous launch file is called

Finally, you can move the robot around from external PC using keyboard

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Setup as a service (automatic on boot)

To bring-up the robot automatically each time you boot the robot (without having to launch the brinup manually), run the following script on the Raspberry Pi

```
cd ~/my_ws/src/scorpion/scripts

./setup_service_penta_bringup_on_boot.sh
```

## Run in simulation

### Using Rviz

You can run the simulation using the command

```
ros2 launch penta_pod penta_sim_rviz.launch.py
```

You can move the robot around using

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Gazebo simulation

You can also run a gazebo simulation using

```
ros2 launch penta_pod penta_gazebo.launch.py
```

## Using devcontainers

You can use devconainers, in VS code using Remote Development plug in, do `Reopen in Container`, then from terminal

```
cd ..

source install/setup.bash

ros2 launch penta_pod penta_sim_rviz.launch.py
```


If rviz crashes, make sure to run in your terminal:

```
xhost +local:docker
```


## Using Docker Compose

You can build the docker image (on ur machine) using the Dockerfile inside Docker folder as the following

```
cd .devcontainer

docker compose up
```

attache the docker container

```
docker ps -a

docker exec -it devcontainer-scorpion-ros2-1 /bin/bash

cd ..

source install/setup.bash

ros2 launch penta_pod penta_pod.launch.py
```