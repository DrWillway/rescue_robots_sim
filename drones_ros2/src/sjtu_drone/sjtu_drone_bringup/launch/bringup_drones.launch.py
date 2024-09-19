#!/usr/bin/env python3

import os
from time import sleep
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def sleep_and_include_launch(context, launch_file_path, sleep_time):
    sleep(sleep_time)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
    )]

def generate_launch_description():
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    neo_simulation2_path = get_package_share_directory('neo_simulation2')
    yolobot_recognition_path = get_package_share_directory('yolobot_recognition')

    print(f"sjtu_drone_bringup_path: {sjtu_drone_bringup_path}")
    print(f"neo_simulation2_path: {neo_simulation2_path}")
    print(f"yolobot_recognition_path: {yolobot_recognition_path}")

    return LaunchDescription([


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sjtu_drone_bringup_path, 'launch', 'drones.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yolobot_recognition_path, 'launch', 'yolo_drones.launch.py')
            )
        ),
        
    ])
