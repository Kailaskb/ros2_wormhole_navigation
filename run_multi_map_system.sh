#!/bin/bash
#
# Multi-Map Navigation System Setup and Launch Script
#
# This script sets up and launches the complete multi-map navigation system:
# 1. Sets up the workspace environment
# 2. Makes Python scripts executable
# 3. Generates map files
# 4. Builds the package
# 5. Sets up the wormhole database
# 6. Launches the full system
#
# Usage: ./run_multi_map_system.sh

# Get directory paths using relative paths
WORKSPACE_DIR="$(realpath "$(dirname "$0")")"
SRC_DIR="${WORKSPACE_DIR}/src/multi_map_nav"
SCRIPTS_DIR="${SRC_DIR}/scripts"

echo "Setting up the multi-map navigation system..."
echo "Workspace directory: ${WORKSPACE_DIR}"
echo "Source directory: ${SRC_DIR}"

# Make sure scripts are executable
chmod +x ${SCRIPTS_DIR}/generate_maps.py
chmod +x ${SCRIPTS_DIR}/setup_wormholes.py
chmod +x ${SCRIPTS_DIR}/verify_map_publishing.py
chmod +x ${SCRIPTS_DIR}/direct_map_publisher.py

# First build without maps
cd "${WORKSPACE_DIR}"
colcon build --packages-select multi_map_nav
source install/setup.bash

# Generate maps in the source directory
echo "Generating maps in the source directory..."
mkdir -p "${SRC_DIR}/maps"

# Run the map generator script
ros2 run multi_map_nav generate_maps.py --ros-args -p output_dir:="${SRC_DIR}/maps"

# Verify maps were created
echo "Verifying map files..."
ls -la "${SRC_DIR}/maps"

# Rebuild to install the maps
cd "${WORKSPACE_DIR}"
colcon build --packages-select multi_map_nav
source install/setup.bash

# Set up the wormhole database
echo "Setting up wormhole database..."
ros2 run multi_map_nav setup_wormholes.py

# Start the direct map publisher in a separate terminal
echo "Starting direct map publisher..."
gnome-terminal --title="Direct Map Publisher" -- bash -c "source ${WORKSPACE_DIR}/install/setup.bash && ros2 run multi_map_nav direct_map_publisher.py; read -p 'Press Enter to close...'"

# Allow time for the map publisher to start
sleep 3

# Check if maps are being published
echo "Checking map topics..."
ros2 topic list | grep map
ros2 topic echo /map_room1 --once &>/dev/null && echo "map_room1 is being published" || echo "ERROR: map_room1 is not being published"
ros2 topic echo /map_room2 --once &>/dev/null && echo "map_room2 is being published" || echo "ERROR: map_room2 is not being published"
ros2 topic echo /map_room3 --once &>/dev/null && echo "map_room3 is being published" || echo "ERROR: map_room3 is not being published"

# Launch the full system
echo "Launching multi-map navigation system..."
ros2 launch multi_map_nav multi_map_full.launch.py

# If the system exits, wait for user input before closing
read -p "Press Enter to exit..."
