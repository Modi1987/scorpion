#!/bin/bash

# Make sure the script is run as root
if [ "$EUID" -ne 0 ]
  then echo "Please run as root (using sudo)"
  exit
fi

DOCKER_COMPOSE_SERVICE_NAME="raspberry-penta-jazzy-ros2"

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