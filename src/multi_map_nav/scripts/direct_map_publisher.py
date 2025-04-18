#!/usr/bin/env python3
"""
Direct Map Publisher for Multi-Map Navigation.

This script directly publishes map data for multi-map navigation, bypassing
the map_server. It's useful for debugging and testing when the map_server
has issues loading or publishing map data.

The script creates and publishes occupancy grid maps for each room, adding
distinctive features to each map to make them visually identifiable.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import time
import os

class DirectMapPublisher(Node):
    """
    Node that directly publishes occupancy grid maps for multi-map navigation.
    
    This node creates and publishes maps for each room in the multi-map system,
    ensuring that maps are available even if the map_server is not functioning
    properly. It's primarily a debugging and fallback mechanism.
    """
    
    def __init__(self):
        """Initialize the direct map publisher node."""
        super().__init__('direct_map_publisher')
        self.get_logger().info('Direct Map Publisher started')
        
        # Create map publishers with QoS settings for reliability
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.map_room1_pub = self.create_publisher(OccupancyGrid, 'map_room1', qos_profile)
        self.map_room2_pub = self.create_publisher(OccupancyGrid, 'map_room2', qos_profile)
        self.map_room3_pub = self.create_publisher(OccupancyGrid, 'map_room3', qos_profile)
        
        # Subscribe to current map name updates
        self.current_map_sub = self.create_subscription(
            String,
            '/multi_map_nav/current_map',
            self.current_map_callback,
            10
        )
        
        # Initial current map
        self.current_map = "room1"
        
        # Timer to publish maps - publish more frequently
        self.timer = self.create_timer(0.2, self.publish_maps)
        
        # Keep track of publish counts for debugging
        self.publish_count = 0
        
        self.get_logger().info('Map Publisher initialized with higher frequency publishing')
        
        # Publish maps immediately on startup
        self.publish_maps()
        
    def current_map_callback(self, msg):
        """
        Handle updates to the current map name.
        
        Args:
            msg (String): Message containing the current map name
        """
        self.current_map = msg.data
        self.get_logger().info(f'Current map changed to: {self.current_map}')
        # Immediately publish maps when the current map changes
        self.publish_maps()
        
    def create_map(self, width=200, height=200, room_name="room"):
        """
        Create a map directly in memory.
        
        Args:
            width (int): Width of the map in pixels
            height (int): Height of the map in pixels
            room_name (str): Room identifier (room1, room2, room3)
            
        Returns:
            OccupancyGrid: Map message with room-specific features
        """
        self.get_logger().info(f'Creating map for {room_name}')
        
        # Create the map message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # Set map metadata
        map_msg.info.resolution = 0.05  # 5cm per pixel
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Create the map data
        # 0 = free space, 100 = occupied, -1 = unknown
        map_data = np.zeros(width * height, dtype=np.int8)
        
        # Add walls (100 = occupied)
        # Top wall
        map_data[0:width] = 100
        # Bottom wall
        map_data[(height-1)*width:height*width] = 100
        # Left wall
        for i in range(height):
            map_data[i*width] = 100
        # Right wall
        for i in range(height):
            map_data[i*width + width - 1] = 100
            
        # Add room-specific features
        if room_name == "room1":
            # Add a doorway on the right wall
            door_start = int(height * 0.4)
            door_end = int(height * 0.6)
            for i in range(door_start, door_end):
                map_data[i*width + width - 1] = 0
                
        elif room_name == "room2":
            # Add doorways on the left and top walls
            # Left doorway (matching room1)
            door_start = int(height * 0.4)
            door_end = int(height * 0.6)
            for i in range(door_start, door_end):
                map_data[i*width] = 0
                
            # Top doorway (to room3)
            door_start = int(width * 0.4)
            door_end = int(width * 0.6)
            for i in range(door_start, door_end):
                map_data[i] = 0
                
        elif room_name == "room3":
            # Add a doorway on the bottom wall
            door_start = int(width * 0.4)
            door_end = int(width * 0.6)
            for i in range(door_start, door_end):
                map_data[(height-1)*width + i] = 0
        
        # Add a recognizable pattern to each room
        if room_name == "room1":
            # Add a cross in the middle
            mid_x = width // 2
            mid_y = height // 2
            thickness = 5
            
            # Horizontal line
            for y in range(mid_y - thickness, mid_y + thickness):
                for x in range(mid_x - 30, mid_x + 30):
                    map_data[y*width + x] = 100
                    
            # Vertical line
            for y in range(mid_y - 30, mid_y + 30):
                for x in range(mid_x - thickness, mid_x + thickness):
                    map_data[y*width + x] = 100
                    
        elif room_name == "room2":
            # Add a circle in the middle
            mid_x = width // 2
            mid_y = height // 2
            radius = 30
            
            for y in range(height):
                for x in range(width):
                    dist = np.sqrt((x - mid_x)**2 + (y - mid_y)**2)
                    if radius - 3 < dist < radius:
                        map_data[y*width + x] = 100
                        
        elif room_name == "room3":
            # Add a square in the middle
            mid_x = width // 2
            mid_y = height // 2
            size = 30
            
            for y in range(mid_y - size, mid_y + size):
                for x in range(mid_x - size, mid_x + size):
                    if (abs(x - mid_x) > size - 3 or abs(y - mid_y) > size - 3):
                        map_data[y*width + x] = 100
        
        # Convert to ROS format
        map_msg.data = map_data.tolist()
        
        return map_msg
        
    def publish_maps(self):
        """Publish all maps."""
        self.publish_count += 1
        
        # Create and publish room1 map
        room1_map = self.create_map(room_name="room1")
        self.map_room1_pub.publish(room1_map)
        
        # Create and publish room2 map
        room2_map = self.create_map(room_name="room2")
        self.map_room2_pub.publish(room2_map)
        
        # Create and publish room3 map
        room3_map = self.create_map(room_name="room3")
        self.map_room3_pub.publish(room3_map)
        
        # Only log occasionally to avoid flooding
        if self.publish_count % 10 == 0:
            self.get_logger().info(f'Published all maps (count: {self.publish_count})')
            self.get_logger().info(f'Map sizes - room1: {len(room1_map.data)}, room2: {len(room2_map.data)}, room3: {len(room3_map.data)}')

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = DirectMapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Final log message
        node.get_logger().info('Direct Map Publisher shutting down')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
