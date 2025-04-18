# ROS 2 Wormhole Navigation

A ROS 2 Humble package enabling seamless navigation across multiple maps using "wormhole" transition points. This system allows robots to operate in facilities with separate map regions (rooms, floors, buildings) while maintaining navigation continuity. Includes VDA 5050 integration for industrial AGVs and fleet management systems.

## Key Features
- 🗺️ Multi-map navigation with automatic transitions
- 🌀 Wormhole-based path planning across separate maps
- 🏭 VDA 5050 industrial standard integration
- 📊 SQLite-backed wormhole database
- 📡 Dynamic map switching and TF management
- 🔎 Visualization tools for wormholes and paths

Designed for warehouses, multi-floor buildings, and facilities with distinct areas that require separate mapping.


# Multi-Map Navigation System

A ROS 2 package for seamless navigation across multiple maps using wormhole transitions with VDA 5050 compatibility.

## Overview

The Multi-Map Navigation System enables robots to navigate across separate map regions by defining transition points (wormholes) between maps. This system provides an action server interface for sending navigation goals to any location on any map, with automatic map transitions when needed. It also implements the VDA 5050 interface standard for industrial AGV fleet management.

## System Architecture

```ascii
+---------------------+     +------------------------+     +--------------------+
| Map Servers         |     | Multi-Map Nav Server   |     | Robot Platform     |
| (room1, room2, etc) | --> | (Navigation + Wormhole | <-- | (TF, Sensors)      |
+---------------------+     | Database)              |     +--------------------+
                            +------------------------+
                                   ^         ^
                                   |         |
                            +------+         +------+
                            |                       |
                     +-------------+         +--------------+
                     | Map Switcher|         | Visualization|
                     | (Map Change)|         | (RViz, etc)  |
                     +-------------+         +--------------+
                           ^                        ^
                           |                        |
                     +-----------------------------+
                     | VDA 5050 Client/Interface   |
                     +-----------------------------+
```

* **Map Servers**: Provide map data for each room/area
* **Multi-Map Navigation Server**: Handles navigation requests across maps
* **Map Switcher**: Manages map transitions between different areas
* **Wormhole Database**: Stores the transition points between maps
* **Visualization**: Tools for monitoring navigation and transitions
* **VDA 5050 Interface**: Translates between VDA 5050 orders and navigation actions

## Features

* **Multi-Map Navigation**: Navigate seamlessly across multiple map regions
* **Map Transitions**: Automatic transitions between maps via wormhole points
* **SQLite Database**: Persistent storage of wormhole connections and navigation states
* **Dynamic Visualization**: Visualization of maps, wormholes, and navigation paths in RViz
* **Path Planning**: Find optimal paths across multiple maps using wormholes
* **VDA 5050 Support**: Standard interface for fleet management systems
* **TF Management**: Proper transformation handling during map transitions
* **Action Server API**: Standard ROS 2 interfaces for navigation commands
* **Robust Error Handling**: Graceful recovery from navigation failures
* **Multiple Map Formats**: Support for various map types and formats
* **Direct Map Publishing**: Fallback mechanism for map server issues
* **Simulation Support**: Built-in support for Gazebo simulation

## Installation

### Prerequisites

* ROS 2 Humble or newer
* SQLite3
* nlohmann_json library

### Building from Source

1. Create a workspace and clone the repository:

```bash
mkdir -p ~/multi_map_ws/src
cd ~/multi_map_ws/src
git clone <repository-url> multi_map_nav
```

2. Install dependencies:

```bash
cd ~/multi_map_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Make scripts executable:

```bash
cd ~/multi_map_ws/src/multi_map_nav/scripts
chmod +x *.py
```

4. Build the package:

```bash
cd ~/multi_map_ws
colcon build --packages-select multi_map_nav
source install/setup.bash
```

## Usage

### Setup Wormhole Database

Before using the system, you need to set up the wormhole database:

```bash
ros2 run multi_map_nav setup_wormholes.py
```

### Generate Maps (if needed)

```bash
ros2 run multi_map_nav generate_maps.py
```

### Launching the Full System

The easiest way to start the entire system is using the launch file:

```bash
ros2 launch multi_map_nav multi_map_full.launch.py
```

Or use the convenience script that handles setup and launching:

```bash
cd ~/multi_map_ws
chmod +x run_multi_map_system.sh
./run_multi_map_system.sh
```

### Verifying Map Publishing

To check if maps are being published correctly:

```bash
ros2 run multi_map_nav verify_map_publishing.py
```

If maps aren't being published by the map server, run the direct map publisher:

```bash
ros2 run multi_map_nav direct_map_publisher.py
```

## Navigation Commands

### Sending a Navigation Goal

To navigate to a specific location on a specific map:

```bash
# Navigate to position (2.0, 3.0) on room2
ros2 launch multi_map_nav send_goal.launch.py map_name:=room2 x:=2.0 y:=3.0
```

### Using Navigation Client

For more control, you can set parameters directly:

```bash
ros2 run multi_map_nav navigation_client --ros-args -p map_name:=room3 -p x:=1.5 -p y:=2.5
```

## VDA 5050 Integration

This system implements the VDA 5050 standard for communication with AGVs (Automated Guided Vehicles), providing a standardized interface for fleet management systems.

### VDA 5050 Features

* **Order Management**: Accept and process VDA 5050 order structures
* **Node/Edge Navigation**: Support for node and edge-based navigation commands
* **Action Handling**: Execute actions attached to nodes (e.g., wait, pick, drop)
* **State Updates**: Publish AGV state updates in VDA 5050 format
* **Error Reporting**: Standardized error reporting structure
* **Visualization**: Visualization of VDA 5050 orders and paths in RViz

### Using VDA 5050 Interface

The VDA 5050 client node provides the interface for receiving orders and publishing state updates:

```bash
ros2 run multi_map_nav vda5050_client
```

Orders can be sent to the `/vda5050/order` topic in JSON format, following the VDA 5050 specification:

```bash
ros2 topic pub /vda5050/order std_msgs/msg/String "data: '{\"orderId\":\"order123\",\"orderUpdateId\":1,\"nodes\":[{\"nodeId\":\"node1\",\"sequenceId\":\"0\",\"nodePosition\":{\"x\":1.0,\"y\":1.0,\"mapId\":\"room1\"},\"released\":true},{\"nodeId\":\"node2\",\"sequenceId\":\"1\",\"nodePosition\":{\"x\":2.0,\"y\":3.0,\"mapId\":\"room2\"},\"actions\":[{\"actionType\":\"waiting\",\"actionId\":\"wait1\"}],\"released\":true}],\"edges\":[{\"edgeId\":\"edge1\",\"sequenceId\":\"0\",\"startNodeId\":\"node1\",\"endNodeId\":\"node2\",\"released\":true}]}'" --once
```

State updates are published to the `/vda5050/state` topic in VDA 5050 format.

## Wormhole Configuration

Wormholes are defined in the `setup_wormholes.py` script. Edit this file to customize the transition points between maps.

Current wormhole configuration:
- **room1 → room2**: Position (4.5, 2.0) to (-4.5, 2.0)
- **room2 → room1**: Position (-4.5, 2.0) to (4.5, 2.0)
- **room2 → room3**: Position (2.0, 4.5) to (2.0, -4.5)
- **room3 → room2**: Position (2.0, -4.5) to (2.0, 4.5)

## Visualizing Wormholes

The system includes a visualization node that shows wormholes in RViz:

```bash
ros2 run multi_map_nav map_visualizer --ros-args -p db_path:=/tmp/wormhole.db
```

## Troubleshooting

### Maps Not Loading

If maps aren't loading correctly:

1. Check that the map files exist:
   ```bash
   ls -la ~/multi_map_ws/install/multi_map_nav/share/multi_map_nav/maps
   ```

2. Run the direct map publisher:
   ```bash
   ros2 run multi_map_nav direct_map_publisher.py
   ```

3. Verify map topics:
   ```bash
   ros2 topic list | grep map
   ros2 topic echo /map_room1 --once
   ```

### Navigation Issues

If the robot isn't navigating between maps:

1. Check the wormhole database:
   ```bash
   sqlite3 /tmp/wormhole.db "SELECT * FROM wormholes;"
   ```

2. Ensure the current map is published:
   ```bash
   ros2 topic echo /multi_map_nav/current_map --once
   ```

3. Check TF transformations:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

## License

Apache License 2.0
