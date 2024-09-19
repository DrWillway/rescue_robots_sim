#!/usr/bin/env python3

import os
from time import sleep  # Import sleep function
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction  # Import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# def get_teleop_controller(context, *_, **kwargs) -> Node:
#     controller = context.launch_configurations["controller"]
#     namespace = kwargs["model_ns"]
    
#     if controller == "joystick":
#         node = Node(
#             package="sjtu_drone_control",
#             executable="teleop_joystick",
#             namespace=namespace,
#             output="screen",
#         )
#     else:
#         node = Node(
#             package="sjtu_drone_control",
#             executable="teleop",
#             namespace=namespace,
#             output="screen",
#             prefix="xterm -e",
#         )

#     return [node]

def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    neo_simulation2_path = get_package_share_directory('neo_simulation2')

    print(f"sjtu_drone_bringup_path: {sjtu_drone_bringup_path}")
    print(f"neo_simulation2_path: {neo_simulation2_path}")

    rviz_path = os.path.join(sjtu_drone_bringup_path, "rviz", "rviz.rviz")
    print(f"rviz_path: {rviz_path}")

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     "controller",
        #     default_value="keyboard",
        #     description="Type of controller: keyboard (default) or joystick",
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'drone1_gazebo.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'drone2_gazebo.launch.py')
            )
        ),
    ])