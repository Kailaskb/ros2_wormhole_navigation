#!/usr/bin/env python3

import numpy as np
import os
import sys
import argparse
import rclpy
from rclpy.node import Node

class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator')
        
        # Declare parameters
        self.declare_parameter('output_dir', '')
        
        # Get the output directory parameter
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        
        if not output_dir:
            # Default to package maps directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            output_dir = os.path.join(os.path.dirname(script_dir), 'maps')
        
        self.get_logger().info(f"Generating maps in: {output_dir}")
        
        # Create directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate maps
        self.generate_maps(output_dir)
        
    def generate_maps(self, maps_dir):
        """Generate maps for all rooms"""
        for room in ['room1', 'room2', 'room3']:
            map_file = os.path.join(maps_dir, f'{room}.pgm')
            self.create_empty_map(map_file)
            self.create_yaml_file(map_file)
            
        self.get_logger().info("All maps generated successfully!")
        # List the contents of the directory
        self.get_logger().info("Maps directory contains:")
        for f in os.listdir(maps_dir):
            self.get_logger().info(f"  {f} - {os.path.getsize(os.path.join(maps_dir, f))} bytes")
    
    def create_empty_map(self, filename, width=200, height=200):
        """Create an empty map with walls around the perimeter."""
        # Create an empty map (255 = free space, 0 = occupied)
        pgm_map = np.ones((height, width), dtype=np.uint8) * 255
        
        # Add walls (0 = occupied)
        # Border walls
        pgm_map[0, :] = 0  # Top wall
        pgm_map[height-1, :] = 0  # Bottom wall
        pgm_map[:, 0] = 0  # Left wall
        pgm_map[:, width-1] = 0  # Right wall
        
        # Write PGM file
        with open(filename, 'wb') as f:
            # Write header
            f.write(b'P5\n')
            f.write(b'# CREATOR: Multi Map Nav\n')
            f.write(f'{width} {height}\n'.encode())
            f.write(b'255\n')
            
            # Write data
            f.write(pgm_map.tobytes())
        
        self.get_logger().info(f"Created map file: {filename}")
        # Make sure file was created with some content
        if os.path.getsize(filename) < 100:
            self.get_logger().warn(f"WARNING: Map file {filename} seems too small!")

    def create_yaml_file(self, pgm_file, resolution=0.05):
        """Create a yaml file for the pgm map file."""
        yaml_file = os.path.splitext(pgm_file)[0] + '.yaml'
        with open(yaml_file, 'w') as f:
            f.write(f"image: {os.path.basename(pgm_file)}\n")
            f.write(f"resolution: {resolution}\n")
            f.write("origin: [-5.0, -5.0, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
            f.write("mode: trinary\n")
        self.get_logger().info(f"Created YAML file: {yaml_file}")

def main(args=None):
    rclpy.init(args=args)
    node = MapGenerator()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
