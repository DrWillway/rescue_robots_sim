from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([       
        Node(
            package='sjtu_drone_control',
            executable='control_robot',
            name='control_robot1',
            output='screen',
            parameters=[{
                'namespace': "robot1",
            }]
        ),

        Node(
            package='sjtu_drone_control',
            executable='control_robot',
            name='control_robot2',
            output='screen',
            parameters=[{
                'namespace': "robot2",
            }]
        )
    ])