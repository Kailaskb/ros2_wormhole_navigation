#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import time
import os
import sys

class MapVerifier(Node):
    def __init__(self):
        super().__init__('map_verifier')
        self.get_logger().info('Map Verifier node started')
        
        # Track which maps we've received
        self.map_received = False
        self.room1_map_received = False
        self.room2_map_received = False
        self.room3_map_received = False
        self.current_map_name = "room1"  # Default
        
        # Create subscription to the main map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Create subscriptions to individual map topics
        self.room1_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map_room1',
            self.room1_map_callback,
            10
        )
        
        self.room2_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map_room2',
            self.room2_map_callback,
            10
        )
        
        self.room3_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map_room3',
            self.room3_map_callback,
            10
        )
        
        # Track current map
        self.current_map_sub = self.create_subscription(
            String,
            '/multi_map_nav/current_map',
            self.current_map_callback,
            10
        )
        
        # Create publishers to help with debugging
        self.test_map_pub = self.create_publisher(
            OccupancyGrid,
            '/test_map',
            10
        )
        
        # Check for map files directly
        self.check_map_files()
        
        # Publish a test map periodically
        self.timer = self.create_timer(1.0, self.publish_test_map)
        
    def check_map_files(self):
        """Check if map files exist and have content"""
        try:
            import subprocess
            from launch_ros.substitutions import FindPackageShare
            from launch.substitutions import PathJoinSubstitution
            from launch.utilities import perform_substitutions
            
            # Use find_package_share to get the maps directory
            pkg_share = perform_substitutions(None, [FindPackageShare('multi_map_nav')])
            maps_dir = os.path.join(pkg_share, 'maps')
            
            self.get_logger().info(f"Checking maps in: {maps_dir}")
            
            if not os.path.exists(maps_dir):
                self.get_logger().error(f"Maps directory not found: {maps_dir}")
                return
                
            # Check all map files
            for room in ['room1', 'room2', 'room3']:
                pgm_path = os.path.join(maps_dir, f"{room}.pgm")
                yaml_path = os.path.join(maps_dir, f"{room}.yaml")
                
                if not os.path.exists(pgm_path):
                    self.get_logger().error(f"Missing PGM file: {pgm_path}")
                else:
                    size = os.path.getsize(pgm_path)
                    self.get_logger().info(f"Found PGM file: {pgm_path}, size: {size} bytes")
                    if size < 100:
                        self.get_logger().warn(f"PGM file seems too small: {size} bytes")
                
                if not os.path.exists(yaml_path):
                    self.get_logger().error(f"Missing YAML file: {yaml_path}")
                else:
                    self.get_logger().info(f"Found YAML file: {yaml_path}")
                    
        except Exception as e:
            self.get_logger().error(f"Error checking map files: {str(e)}")
        
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info(f'Main map received! Size: {len(msg.data)}')
            self.get_logger().info(f'Map info - width: {msg.info.width}, height: {msg.info.height}')
            self.get_logger().info(f'Map frame_id: {msg.header.frame_id}')
            self.map_received = True
        else:
            self.get_logger().debug('Map update received')
    
    def room1_map_callback(self, msg):
        if not self.room1_map_received:
            self.get_logger().info(f'Room1 map received! Size: {len(msg.data)}')
            self.room1_map_received = True
    
    def room2_map_callback(self, msg):
        if not self.room2_map_received:
            self.get_logger().info(f'Room2 map received! Size: {len(msg.data)}')
            self.room2_map_received = True
    
    def room3_map_callback(self, msg):
        if not self.room3_map_received:
            self.get_logger().info(f'Room3 map received! Size: {len(msg.data)}')
            self.room3_map_received = True
            
    def current_map_callback(self, msg):
        self.current_map_name = msg.data
        self.get_logger().info(f'Current map is: {self.current_map_name}')
    
    def publish_test_map(self):
        # Create a simple test map
        test_map = OccupancyGrid()
        test_map.header.stamp = self.get_clock().now().to_msg()
        test_map.header.frame_id = "map"
        test_map.info.resolution = 0.05
        test_map.info.width = 20
        test_map.info.height = 20
        test_map.info.origin.position.x = -5.0
        test_map.info.origin.position.y = -5.0
        test_map.info.origin.orientation.w = 1.0
        
        # Create a simple map - all free space
        test_map.data = [0] * (test_map.info.width * test_map.info.height)
        
        # Add a border of obstacles
        for i in range(test_map.info.width):
            test_map.data[i] = 100  # Top border
            test_map.data[(test_map.info.height-1) * test_map.info.width + i] = 100  # Bottom border
        
        for i in range(test_map.info.height):
            test_map.data[i * test_map.info.width] = 100  # Left border
            test_map.data[i * test_map.info.width + test_map.info.width - 1] = 100  # Right border
        
        # Publish the test map
        self.test_map_pub.publish(test_map)
        self.get_logger().debug('Published test map')

def main(args=None):
    rclpy.init(args=args)
    node = MapVerifier()
    
    try:
        # Run for 15 seconds
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 15.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Print interim status every 5 seconds
            if time.time() - start_time > 5.0 and (time.time() - start_time) % 5.0 < 0.1:
                node.get_logger().info('=== STATUS UPDATE ===')
                node.get_logger().info(f'Main map received: {node.map_received}')
                node.get_logger().info(f'Room1 map received: {node.room1_map_received}')
                node.get_logger().info(f'Room2 map received: {node.room2_map_received}')
                node.get_logger().info(f'Room3 map received: {node.room3_map_received}')
                node.get_logger().info(f'Current map: {node.current_map_name}')
                node.check_map_files()  # Check map files again during update
            
        # Final status report
        node.get_logger().info('=== FINAL STATUS ===')
        node.get_logger().info(f'Main map received: {node.map_received}')
        node.get_logger().info(f'Room1 map received: {node.room1_map_received}')
        node.get_logger().info(f'Room2 map received: {node.room2_map_received}')
        node.get_logger().info(f'Room3 map received: {node.room3_map_received}')
        node.get_logger().info(f'Current map: {node.current_map_name}')
        
        if not node.map_received:
            node.get_logger().error('No map data received on /map topic!')
        else:
            node.get_logger().info('Map verification completed successfully')
            
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
