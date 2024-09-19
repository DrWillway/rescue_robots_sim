## Aerial and terrain drones search and rescue project in gazebo with ROS2

- Install ROS2 humble

- Install gazebo-11

- Install ros2 required packages 

```
export ROS_DISTRO=humble
sudo apt install ros-$ROS_DISTRO-gazebo*
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-*
sudo apt install ros-$ROS_DISTRO-slam-toolbox
sudo apt-get install ros-$ROS_DISTRO-topic-tools
sudo apt-get install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-rqt-joint-trajectory-controller ros-$ROS_DISTRO-moveit-*
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros2-control
sudo apt-get install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-rqt-joint-trajectory-controller ros-$ROS_DISTRO-moveit-*
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
sudo apt install xterm
```

Clone this repo and:
```
cd drones_project/drones_ros2
sudo rosdep init 
rosdep update
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
Add to ~/.bashrc
```
source /opt/ros/humble/setup.bash
source ~/drones_ros2/install/setup.bash # full path to installed location of this repo 
source /usr/share/gazebo/setup.bash
```

Build the packages with gazebo and drones:
```
cd drones_project/drones_ros2
colcon build --symlink-install
```

### Run drones in gazebo

```
cd drones_project/drones_ros2 && source install/setup.bash
ros2 launch sjtu_drone_bringup drones.launch.py
```
__NOTE:__ It may take a while to open the world.

#### Run control (explore with zigzag movements)

In another terminal run:

```
cd drones_project/drones_ros2 && source install/setup.bash
ros2 launch sjtu_drone_control control_drones.launch.py
```
You should see gazebo world with 2 UAVs flying.

### To observe drones camera images:

```
rviz2
```

Add -> by topic -> drone1 -> bottom -> image_raw

## Run detections - yolo

Install requirements:

```
sudo apt install python3-pip
cd drones_ros2
pip3 install -r requirements_yolo.txt  # install
```

This will start detections of humans from images of 2 drones:
```
ros2 launch yolobot_recognition yolo_drones.launch.py
```

# Run agents

Run bash script that starts spade agents responsible for two drones navigation and human searching, and two rescue robots.

./drones_ros2/src/spade/agents/start_agents.sh




# Tests with two terrain robots navigation (small vehicles)

export MY_ROBOT=mp_500
export MAP_NAME=neo_track1
export Number_of_Robots=2

```
ros2 launch neo_simulation2 multi_robot_simulation.launch.py
```

```
ros2 launch neo_simulation2 multi_robot_navigation.py
```

```
ros2 topic pub /robot1/goal_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -7.0, y: 2.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

```
ros2 topic pub /robot2/goal_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 2.0, y: 2.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```


# Demo

ros2 launch sjtu_drone_bringup bringup_land_robots.launch.py

ros2 launch sjtu_drone_bringup bringup_drones.launch.py

ros2 launch sjtu_drone_control control_drones.launch.py

cd /root/drones_ros2/src/spade/agents && ./start_agents.sh