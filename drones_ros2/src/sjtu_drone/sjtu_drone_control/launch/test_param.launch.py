from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'param_name',
        #     default_value='default_value',
        #     description='Description of the parameter'
        # ),
        
        Node(
            package='sjtu_drone_control',
            executable='test_param',
            name='test_param',
            output='screen',
            parameters=[{
                'namespace': "drone1", #LaunchConfiguration('param_name')
                'x_start': -8.5,
                'y_start': "-7.5",
                'width': "7.0",
                'depth': "7.0" 
            }]
        )
    ])