#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Infer the workspace directory from the script's location
WORKSPACE_SOURCE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Add source to .bashrc if not already there
if ! grep -Fxq "source ${WORKSPACE_SOURCE_DIR}/install/setup.bash" ~/.bashrc; then
    echo "source ${WORKSPACE_SOURCE_DIR}/install/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 workspace sourcing to .bashrc"
else
    echo "ROS 2 workspace already sourced in .bashrc"
fi

# Reload bashrc
source ~/.bashrc

# Create a systemd service for ROS 2 launch
SERVICE_FILE="/etc/systemd/system/pentapod_bringup.service"
sudo bash -c "cat > $SERVICE_FILE" <<EOL
[Unit]
Description=ROS 2 Launch Service
After=network.target

[Service]
ExecStart=/usr/bin/bash -c 'source ${WORKSPACE_SOURCE_DIR}/install/setup.bash && ros2 launch penta_pod realhardware_bringup.launch.py'
WorkingDirectory=${WORKSPACE_SOURCE_DIR}

Restart=on-failure
User=$(whoami)
Environment=ROS_DOMAIN_ID=91  # Set if needed
# Set environment variables
Environment="ROS_DOMAIN_ID=42"
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/modi/.Xauthority"

TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOL

# Reload systemd and enable the service
sudo systemctl daemon-reload
sudo systemctl enable pentapod_bringup.service

echo "ROS 2 service has been created and enabled."
