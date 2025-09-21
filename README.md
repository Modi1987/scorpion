[![Powered by Dev Containers](https://img.shields.io/badge/Powered%20by-Dev%20Containers-blue?logo=devcontainers&logoColor=white)](https://containers.dev/)


# Pentapod

This is a ROS2 software for the following older project [youtube video available here.](https://youtu.be/kcvJR5mcb1o?si=lxt_06UO4189CPcX)


## Setting up on real robot (Raspberry PI)

Follow the steps:

- Install docker and docker compose on the Raspberry-PI. 

- After the installation, do not forget to add user to the docker group

```
# Add your user to the docker group
sudo usermod -aG docker $USER

# Apply changes (log out and back in, or run:)
newgrp docker
```

Then, restart the Raspberry PI to apply the changes

- Create your workspace folder

```
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws/src
```

- Clone the repo

```
git clone https://github.com/Modi1987/polypod.git

cd polypod

git checkout poc/jazzy
```

- Then, do the installation using the installation scripts (this script will build your workspace inside docker on Raspberry PI and autostart the robot on boot)

```
cd ~/ros2_ws/src/polypod/.raspberry_container

sudo ./setup_on_raspberry.sh
```

- After the installation is done reboot the robot

- Enjoy