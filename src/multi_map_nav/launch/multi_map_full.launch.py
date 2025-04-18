"""
Main launch file for the multi-map navigation system.

This launch file sets up and starts all components of the multi-map navigation
system, including:
- Map servers for each room
- Wormhole database setup
- Navigation server
- Map switcher
- Visualization
- TF publisher
- Robot state publisher
- RViz for visualization

It provides a complete system for navigating across multiple maps using
wormholes for transitions between maps.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """Generate the launch description for the full multi-map system."""
    # Launch arguments
    db_path = LaunchConfiguration('db_path')
    current_map = LaunchConfiguration('current_map')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Paths
    pkg_share = FindPackageShare('multi_map_nav')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'db_path',
            default_value='/tmp/wormhole.db',
            description='Path to the SQLite database file'
        ),
        
        DeclareLaunchArgument(
            'current_map',
            default_value='room1',
            description='Current map name'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'multi_map_nav.rviz']),
            description='RViz configuration file'
        ),
        
        # Include map loading launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'load_maps.launch.py'])
            ])
        ),
        
        # Include multi_map_nav core launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'multi_map_nav.launch.py'])
            ]),
            launch_arguments={
                'db_path': db_path,
                'current_map': current_map,
            }.items()
        ),
        
        # Setup wormholes
        Node(
            package='multi_map_nav',
            executable='setup_wormholes.py',
            name='setup_wormholes',
            parameters=[{'db_path': db_path}],
            output='screen'
        ),
        
        # Map visualization node
        Node(
            package='multi_map_nav',
            executable='map_visualizer',
            name='map_visualizer',
            output='screen',
            parameters=[{'db_path': db_path}]
        ),
        
        # TF Publisher node
        Node(
            package='multi_map_nav',
            executable='tf_publisher_node',
            name='tf_publisher_node',
            output='screen'
        ),
        
        # Robot state publisher node
        Node(
            package='multi_map_nav',
            executable='robot_state_publisher_node',
            name='robot_state_publisher_node',
            output='screen'
        ),
        
        # RViz
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
