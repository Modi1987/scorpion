#!/bin/bash

# Update package list
sudo apt update

# Install pip if not already installed
sudo apt install python3-pip

# Install RPi.GPIO
sudo pip install RPi.GPIO

# Install adafruit-blinka
sudo pip install adafruit-blinka

# Install i2c packages
sudo apt-get install python3-smbus
sudo apt-get install i2c-tools
