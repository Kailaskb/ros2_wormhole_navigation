from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments for the goal
    map_name = LaunchConfiguration('map_name')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    qx = LaunchConfiguration('qx')
    qy = LaunchConfiguration('qy')
    qz = LaunchConfiguration('qz')
    qw = LaunchConfiguration('qw')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'map_name',
            default_value='room2',
            description='Target map name'
        ),
        
        DeclareLaunchArgument(
            'x',
            default_value='2.0',
            description='X coordinate for target pose'
        ),
        
        DeclareLaunchArgument(
            'y',
            default_value='3.0',
            description='Y coordinate for target pose'
        ),
        
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Z coordinate for target pose'
        ),
        
        DeclareLaunchArgument(
            'qx',
            default_value='0.0',
            description='X component of quaternion orientation'
        ),
        
        DeclareLaunchArgument(
            'qy',
            default_value='0.0',
            description='Y component of quaternion orientation'
        ),
        
        DeclareLaunchArgument(
            'qz',
            default_value='0.0',
            description='Z component of quaternion orientation'
        ),
        
        DeclareLaunchArgument(
            'qw',
            default_value='1.0',
            description='W component of quaternion orientation'
        ),
        
        # Navigation client node
        Node(
            package='multi_map_nav',
            executable='navigation_client',
            name='navigation_client',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_name': map_name,
                'x': x_pos,
                'y': y_pos,
                'z': z_pos,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw
            }]
        )
    ])
