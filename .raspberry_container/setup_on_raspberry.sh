#!/bin/bash

# Make sure the script is run as root
if [ "$EUID" -ne 0 ]
  then echo "Please run as root (using sudo)"
  exit
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

if ! command -v docker-compose &> /dev/null
then
    echo "Docker Compose could not be found, please install Docker Compose."
    exit
fi

# Check if docker works without sudo
if ! docker info &> /dev/null
then
    echo "Docker does not seem to work without sudo. Please configure Docker to run without sudo."
    exit
fi

# Docker compose up the container
docker compose up -d raspberry-penta-jazzy-ros2
