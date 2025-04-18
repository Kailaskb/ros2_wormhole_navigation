from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    db_path = LaunchConfiguration('db_path')
    current_map = LaunchConfiguration('current_map')
    
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
        
        # Navigation server
        Node(
            package='multi_map_nav',
            executable='navigation_server',
            name='multi_map_navigation_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'db_path': db_path},
                {'current_map': current_map}
            ]
        ),
        
        # Map switcher node
        Node(
            package='multi_map_nav',
            executable='map_switcher',
            name='map_switcher',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('map', '/map')  # Ensure map is published to global namespace
            ]
        ),
        
        # VDA5050 client (optional)
        Node(
            package='multi_map_nav',
            executable='vda5050_client',
            name='vda5050_client',
            output='screen',
            emulate_tty=True,
        )
    ])
