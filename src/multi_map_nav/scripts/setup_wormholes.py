#!/usr/bin/env python3
"""
Setup wormholes for multi-map navigation.

This script creates and configures wormholes between different maps in the
multi-map navigation system. Wormholes represent transition points that allow
the robot to navigate between separate maps.

The script:
1. Connects to the SQLite database
2. Defines wormhole connections between maps
3. Adds these connections to the database
4. Verifies that the wormholes were correctly added

Usage:
    ros2 run multi_map_nav setup_wormholes.py [--db_path DB_PATH]
"""

import sys
import sqlite3
import argparse
import rclpy
from rclpy.node import Node
import os

class WormholeSetup(Node):
    """
    Node for setting up wormholes between maps.
    
    This class handles the creation and configuration of wormholes in the
    SQLite database. It defines the transition points between different maps
    that allow the robot to navigate through the entire environment.
    """
    
    def __init__(self):
        """Initialize the wormhole setup node."""
        super().__init__('wormhole_setup')
        
        # Declare parameters
        self.declare_parameter('db_path', '/tmp/wormhole.db')
        
        # Get parameters
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value
        
        self.get_logger().info(f"Using database at: {self.db_path}")
        
        # Run the setup process
        success = self.setup_wormholes()
        if success:
            self.get_logger().info("Wormhole setup completed successfully!")
        else:
            self.get_logger().error("Wormhole setup failed!")
    
    def setup_wormholes(self):
        """
        Set up wormholes between maps.
        
        Creates wormhole connections between different maps:
        - room1 <-> room2
        - room2 <-> room3
        
        Returns:
            bool: True if setup was successful, False otherwise
        """
        try:
            # Connect to the database
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Create wormholes table if it doesn't exist
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS wormholes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                source_map TEXT NOT NULL,
                target_map TEXT NOT NULL,
                source_x REAL NOT NULL,
                source_y REAL NOT NULL,
                source_z REAL NOT NULL,
                source_qx REAL NOT NULL,
                source_qy REAL NOT NULL,
                source_qz REAL NOT NULL,
                source_qw REAL NOT NULL,
                target_x REAL NOT NULL,
                target_y REAL NOT NULL,
                target_z REAL NOT NULL,
                target_qx REAL NOT NULL,
                target_qy REAL NOT NULL,
                target_qz REAL NOT NULL,
                target_qw REAL NOT NULL,
                transition_cost REAL NOT NULL,
                UNIQUE(source_map, target_map)
            )
            ''')
            conn.commit()
            
            # Define wormholes
            wormholes = [
                # Connection from room1 to room2
                {
                    'source_map': 'room1',
                    'target_map': 'room2',
                    'source_x': 4.5, 'source_y': 2.0, 'source_z': 0.0,
                    'source_qx': 0.0, 'source_qy': 0.0, 'source_qz': 0.0, 'source_qw': 1.0,
                    'target_x': -4.5, 'target_y': 2.0, 'target_z': 0.0,
                    'target_qx': 0.0, 'target_qy': 0.0, 'target_qz': 0.0, 'target_qw': 1.0,
                    'transition_cost': 1.0
                },
                # Connection from room2 to room1
                {
                    'source_map': 'room2',
                    'target_map': 'room1',
                    'source_x': -4.5, 'source_y': 2.0, 'source_z': 0.0,
                    'source_qx': 0.0, 'source_qy': 0.0, 'source_qz': 0.0, 'source_qw': 1.0,
                    'target_x': 4.5, 'target_y': 2.0, 'target_z': 0.0,
                    'target_qx': 0.0, 'target_qy': 0.0, 'target_qz': 0.0, 'target_qw': 1.0,
                    'transition_cost': 1.0
                },
                # Connection from room2 to room3
                {
                    'source_map': 'room2',
                    'target_map': 'room3',
                    'source_x': 2.0, 'source_y': 4.5, 'source_z': 0.0,
                    'source_qx': 0.0, 'source_qy': 0.0, 'source_qz': 0.0, 'source_qw': 1.0,
                    'target_x': 2.0, 'target_y': -4.5, 'target_z': 0.0,
                    'target_qx': 0.0, 'target_qy': 0.0, 'target_qz': 0.0, 'target_qw': 1.0,
                    'transition_cost': 1.0
                },
                # Connection from room3 to room2
                {
                    'source_map': 'room3',
                    'target_map': 'room2',
                    'source_x': 2.0, 'source_y': -4.5, 'source_z': 0.0,
                    'source_qx': 0.0, 'source_qy': 0.0, 'source_qz': 0.0, 'source_qw': 1.0,
                    'target_x': 2.0, 'target_y': 4.5, 'target_z': 0.0,
                    'target_qx': 0.0, 'target_qy': 0.0, 'target_qz': 0.0, 'target_qw': 1.0,
                    'transition_cost': 1.0
                }
            ]
            
            # Insert wormholes into the database
            for wormhole in wormholes:
                cursor.execute('''
                INSERT OR REPLACE INTO wormholes 
                (source_map, target_map, 
                 source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw,
                 target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw,
                 transition_cost)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', (
                    wormhole['source_map'], wormhole['target_map'],
                    wormhole['source_x'], wormhole['source_y'], wormhole['source_z'],
                    wormhole['source_qx'], wormhole['source_qy'], wormhole['source_qz'], wormhole['source_qw'],
                    wormhole['target_x'], wormhole['target_y'], wormhole['target_z'],
                    wormhole['target_qx'], wormhole['target_qy'], wormhole['target_qz'], wormhole['target_qw'],
                    wormhole['transition_cost']
                ))
            
            conn.commit()
            
            # Verify that wormholes were added
            cursor.execute('SELECT COUNT(*) FROM wormholes')
            count = cursor.fetchone()[0]
            self.get_logger().info(f"Number of wormholes in database: {count}")
            
            # List all wormholes
            cursor.execute('SELECT source_map, target_map, source_x, source_y, target_x, target_y FROM wormholes')
            for row in cursor.fetchall():
                self.get_logger().info(f"Wormhole: {row[0]} -> {row[1]} at ({row[2]}, {row[3]}) -> ({row[4]}, {row[5]})")
            
            conn.close()
            return True
            
        except sqlite3.Error as e:
            self.get_logger().error(f"SQLite error: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error setting up wormholes: {e}")
            return False

def main():
    """Main function."""
    rclpy.init()
    
    node = WormholeSetup()
    
    # We don't need to spin since setup is done in constructor
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
