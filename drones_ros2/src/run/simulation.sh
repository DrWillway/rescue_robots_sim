#!/bin/bash

# Launch drones.launch.py from sjtu_drone_bringup package in background

export MY_ROBOT=mp_500
export MAP_NAME=neo_track1
export Number_of_Robots=2

ros2 launch sjtu_drone_bringup bringup_land_robots.launch.py &
ros2 launch sjtu_drone_bringup bringup_drones.launch.py
ros2 launch sjtu_drone_control control_drones.launch.py

# /root/MEIA-8-T4/drones_ros2/src/spade/agents/start_agents.sh