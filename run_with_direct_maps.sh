#!/bin/bash
#
# Multi-Map Navigation with Direct Map Publishing
#
# This script runs the direct map publisher first and then launches
# the full navigation system to ensure maps are always available.
#

WORKSPACE_DIR="$(realpath "$(dirname "$0")")"

# Source the workspace
cd "${WORKSPACE_DIR}"
source install/setup.bash

# Start the direct map publisher in the background
echo "Starting direct map publisher..."
ros2 run multi_map_nav direct_map_publisher.py > /tmp/direct_map_publisher.log 2>&1 &
MAP_PUBLISHER_PID=$!

# Make sure to kill the direct map publisher when the script exits
trap "kill $MAP_PUBLISHER_PID 2>/dev/null" EXIT

# Allow time for the map publisher to initialize
sleep 2

# Check if maps are being published
echo "Checking map topics..."
ros2 topic list | grep map
echo "Map publisher is running with PID: $MAP_PUBLISHER_PID"

# Now run the main system
echo "Launching the multi-map navigation system..."
ros2 launch multi_map_nav multi_map_full.launch.py

echo "System has exited. Press Enter to close this terminal..."
read
