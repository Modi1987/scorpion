#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Infer the workspace directory from the script's location
WORKSPACE_SOURCE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Workspace source dir:"
echo "$WORKSPACE_SOURCE_DIR"

# Build the workspace
echo "Building the workspace..."
cd "$WORKSPACE_SOURCE_DIR"/.. || exit
colcon build

echo "Workspace setup complete for $WORKSPACE_SOURCE_DIR."