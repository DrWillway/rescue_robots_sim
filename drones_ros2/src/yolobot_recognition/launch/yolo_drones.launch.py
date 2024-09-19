import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='drone1',
                                          description='Namespace for the node')

    # Node for drone1
    drone1_node = Node(
        package='yolobot_recognition',
        executable='ros_recognition_yolo',
        name='ros_recognition_yolo_drone1',
        namespace='',
        parameters=[{'namespace': 'drone1', 'weights': 'yolov5s.pt'}],
        output='screen'
    )

    # Node for drone2
    drone2_node = Node(
        package='yolobot_recognition',
        executable='ros_recognition_yolo',
        name='ros_recognition_yolo_drone2',
        namespace='',
        parameters=[{'namespace': 'drone2', 'weights': 'yolov5s.pt'}],
        output='screen'
    )

    # Add nodes and arguments to launch description
    ld.add_action(namespace_arg)
    ld.add_action(drone1_node)
    ld.add_action(drone2_node)

    return ld
