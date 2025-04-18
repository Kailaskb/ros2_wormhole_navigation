from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Paths
    pkg_share = FindPackageShare('multi_map_nav')
    default_map_dir = PathJoinSubstitution([pkg_share, 'maps'])
    
    # Launch arguments
    map_dir = LaunchConfiguration('map_dir')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'map_dir',
            default_value=default_map_dir,
            description='Directory containing map files'
        ),
        
        # Log info for debugging
        LogInfo(msg=['Loading maps from directory: ', map_dir]),
        
        # Check if map files exist (just for debugging)
        ExecuteProcess(
            cmd=['ls', '-la', PathJoinSubstitution([pkg_share, 'maps'])],
            output='screen'
        ),
        
        # Map Server for Room 1
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_room1',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'yaml_filename': PathJoinSubstitution([map_dir, 'room1.yaml']),
                'topic': 'map_room1',
                'frame_id': 'map',
                'use_sim_time': False
            }]
        ),
        
        # Map Server for Room 2  
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_room2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'yaml_filename': PathJoinSubstitution([map_dir, 'room2.yaml']),
                'topic': 'map_room2',
                'frame_id': 'map',
                'use_sim_time': False
            }]
        ),
        
        # Map Server for Room 3
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_room3',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'yaml_filename': PathJoinSubstitution([map_dir, 'room3.yaml']),
                'topic': 'map_room3',
                'frame_id': 'map',
                'use_sim_time': False
            }]
        ),

        # Map Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_maps',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'autostart': True,
                'node_names': ['map_server_room1', 'map_server_room2', 'map_server_room3']
            }]
        )
    ])
