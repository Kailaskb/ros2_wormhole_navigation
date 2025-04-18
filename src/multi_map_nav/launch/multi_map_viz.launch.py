from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch arguments
    db_path = LaunchConfiguration('db_path')
    current_map = LaunchConfiguration('current_map')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    
    # Paths
    pkg_share = FindPackageShare('multi_map_nav')
    gazebo_pkg_share = FindPackageShare('gazebo_ros')
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_pkg_share, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items()
    )
    
    # Launch multi_map_nav
    multi_map_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'multi_map_nav.launch.py'])
        ]),
        launch_arguments={
            'db_path': db_path,
            'current_map': current_map,
        }.items()
    )
    
    # Map visualization node
    map_viz_node = Node(
        package='multi_map_nav',
        executable='map_visualizer',
        name='map_visualizer',
        output='screen',
        parameters=[{'db_path': db_path}]
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'db_path',
            default_value='/tmp/wormhole.db',
            description='Path to the SQLite database file'
        ),
        
        DeclareLaunchArgument(
            'current_map',
            default_value='map1',
            description='Current map name'
        ),
        
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'multi_map_nav.rviz']),
            description='RViz configuration file'
        ),
        
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([pkg_share, 'worlds', 'multi_room.world']),
            description='Gazebo world file'
        ),
        
        # Include launch files
        gazebo_launch,
        multi_map_nav_launch,
        rviz_node,
        map_viz_node
    ])
