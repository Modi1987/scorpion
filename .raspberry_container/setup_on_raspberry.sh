#!/bin/bash

# Make sure the script is run as root
if [ "$EUID" -ne 0 ]
  then echo "Please run as root (using sudo)"
  exit
fi

DOCKER_COMPOSE_SERVICE_NAME="raspberry-penta-jazzy-ros2"

# Check for service native pentapod service, and stop it if it exists
NATIVE_SERVICE_NAME="pentapod_bringup.service"
NATIVE_SERVICE_PATH="/etc/systemd/system/$NATIVE_SERVICE_NAME"
if [ -f $NATIVE_SERVICE_PATH ]; then
    echo "Stopping and disabling existing native service: $NATIVE_SERVICE_NAME"
    sudo systemctl stop $NATIVE_SERVICE_NAME
    sudo systemctl disable $NATIVE_SERVICE_NAME
    sudo rm $NATIVE_SERVICE_PATH
    sudo systemctl daemon-reload
fi

# Make sure vcs is installed
apt-get update
apt-get install -y python3-vcstool
apt-get install -y git

# VSC import
vcs import ../../ < ./vcs_repos.yaml

# Check if docker and docker compose are installed otherwise ask to install
if ! command -v docker &> /dev/null
then
    echo "Docker could not be found, please install Docker."
    exit
fi

# Check if docker works without sudo
if ! docker info &> /dev/null
then
    echo "Docker does not seem to work without sudo. Please configure Docker to run without sudo."
    exit
fi

# Docker compose up the container
if command -v docker &> /dev/null && docker compose version &> /dev/null; then
    docker compose up -d $DOCKER_COMPOSE_SERVICE_NAME
elif command -v docker-compose &> /dev/null; then
    docker-compose up -d $DOCKER_COMPOSE_SERVICE_NAME
else
    echo "Docker Compose not found. Please install either 'docker compose' (plugin) or 'docker-compose' (standalone)."
    exit 1
fi

# Add progress bar while building
echo "Setting up the workspace inside the docker container. This may take a while..."
CONTAINER_NAME=$(docker ps --filter "name=$DOCKER_COMPOSE_SERVICE_NAME" --format "{{.Names}}")
# Wait until the container is fully built
while [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME)" != "true" ]; do
    echo -n "."
    sleep 5
done

# Wait until the ROS nodes are launched (indicated by the presence of a .ready file)
echo "Waiting for ROS2 workspace to be built and nodes to start inside the container..."
while ! docker exec "$CONTAINER_NAME" [ -f /root/ros_ws/.ready ]; do
    echo -n "."
    sleep 5
done
echo "Installation complete!"
