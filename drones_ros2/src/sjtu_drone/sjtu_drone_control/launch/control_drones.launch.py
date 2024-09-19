from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([       
        Node(
            package='sjtu_drone_control',
            executable='control_drones',
            name='control_drone1',
            output='screen',
            parameters=[{
                'namespace': "drone2",
                'x_start': -8.5,
                'y_start': -7.5,
                'width': 7.0,
                'depth': 7.0 
            }]
        ),

        Node(
            package='sjtu_drone_control',
            executable='control_drones',
            name='control_drone2',
            output='screen',
            parameters=[{
                'namespace': "drone1",
                'x_start': 0.0,
                'y_start': -7.5,
                'width': 7.0,
                'depth': 7.0 
            }]
        )
    ])