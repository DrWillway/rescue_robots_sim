import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, radians, pi

import math

class GroundRobotController(Node):
    def __init__(self):
        super().__init__('ground_robot_controller')
        
        # Declare and get the namespace parameter
        self.declare_parameter('namespace', 'robot1')
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        # Define topic names with the namespace
        target_pose_topic = f'{namespace}/goal_pose'
        odom_topic = f'{namespace}/odom'
        cmd_vel_topic = f'{namespace}/cmd_vel'
        scan_topic = f'{namespace}/scan'

        # Initialize subscribers and publishers with the namespace
        self.target_pose_subscriber = self.create_subscription(PoseStamped, target_pose_topic, self.target_pose_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        
        self.target_pose = None
        self.current_pose = None
        self.state = 'IDLE'

        self.rotation_accumulated = 0.0
        self.target_rotation = radians(30)
        self.forward_distance = 1.0  # Distance to move forward after avoiding obstacle
        self.forward_start_position = None

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose
        self.state = 'MOVE_TO_POSITION'
        if self.target_pose is not None:
            target_yaw = self.get_yaw(self.target_pose.orientation)
            self.move_to_position(self.target_pose.position, target_yaw)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.target_pose is not None:
            if self.state != 'TURN_AWAY_FROM_OBSTACLE':
                target_yaw = self.get_yaw(self.target_pose.orientation)
                self.move_to_position(self.target_pose.position, target_yaw)

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_readings = len(ranges)
        front_ranges = ranges[num_readings // 4: 3 * num_readings // 4]
        min_range = min(front_ranges)

        if self.current_pose is not None:
            if min_range < 1.0 or self.state == 'TURN_AWAY_FROM_OBSTACLE':
                # Obstacle detected in front, implement avoidance behavior
                vel_msg = Twist()
                vel_msg.linear.x = 0.0  # Stop linear motion

                # Calculate direction away from the obstacle
                desired_yaw = self.get_yaw(self.current_pose.orientation) + math.pi
                if self.state != 'TURN_AWAY_FROM_OBSTACLE':
                    self.turn_start_yaw = self.get_yaw(self.current_pose.orientation)  # Record initial yaw for turning
                    self.state = 'TURN_AWAY_FROM_OBSTACLE'

                # Determine turn direction based on obstacle position
                if self.obstacle_on_right(front_ranges):
                    turn_direction = -1.0  # Turn right
                else:
                    turn_direction = 1.0  # Turn left

                # Calculate how much we have turned
                current_yaw = self.get_yaw(self.current_pose.orientation)
                turned_amount = abs(current_yaw - self.turn_start_yaw)

                # Check if turned less than 50 degrees
                if turned_amount < math.radians(50):
                    vel_msg.angular.z = 0.5 * turn_direction
                else:
                    # Stop turning once turned more than 50 degrees
                    vel_msg.angular.z = 0.0
                    self.state = 'MOVE_FORWARD_AFTER_AVOIDANCE'

                self.vel_publisher.publish(vel_msg)
            else:
                # No obstacle detected, proceed with navigation
                if self.target_pose is not None and self.state != 'TURN_TO_TARGET':
                    target_distance = self.calculate_distance(self.current_pose.position, self.target_pose.position)
                    if target_distance > 1.5:  # Only rotate if target is far enough
                        self.rotate_towards_target()

    def rotate_towards_target(self):
        if self.current_pose is not None and self.target_pose is not None:
            current_yaw = self.get_yaw(self.current_pose.orientation)
            target_yaw = self.get_yaw(self.target_pose.orientation)
            yaw_difference = self.normalize_angle(target_yaw - current_yaw)

            vel_msg = Twist()
            if abs(yaw_difference) > radians(8):  # Adjust yaw if the difference is significant
                vel_msg.angular.z = 0.5 if yaw_difference > 0 else -0.5
                self.state = 'TURN_TO_TARGET'
            else:
                # Stop turning
                vel_msg.angular.z = 0.0
                self.state = 'MOVE_TO_POSITION'

            self.vel_publisher.publish(vel_msg)
            
    def move_to_position(self, target, target_yaw):
        if self.current_pose is None:
            return False

        current_position = self.current_pose.position
        distance_to_target = self.calculate_distance(current_position, target)
        vel_msg = Twist()

        if distance_to_target < 0.5:  # Threshold distance to consider the target reached
            current_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_difference = self.normalize_angle(target_yaw - current_yaw)

            self.state = 'IDLE'
            self.vel_publisher.publish(Twist())  # Stop the robot
            return True
        else:
            angle_to_target = atan2(target.y - current_position.y, target.x - current_position.x)
            current_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_difference = self.normalize_angle(angle_to_target - current_yaw)

            if self.state == 'MOVE_FORWARD_AFTER_AVOIDANCE':
                if self.forward_start_position is not None:
                    distance_traveled = self.calculate_distance(self.forward_start_position, current_position)
                    if distance_traveled < self.forward_distance:
                        vel_msg.linear.x = 0.5  # Move forward
                    else:
                        vel_msg.linear.x = 1.5 if distance_to_target > 1.0 else 0.3
                        self.state = 'MOVE_TO_TARGET'
            else:
                vel_msg.linear.x = 1.5 if distance_to_target > 1.0 else 0.3
                self.state = 'MOVE_TO_TARGET'
            if self.state != 'MOVE_FORWARD_AFTER_AVOIDANCE':
                if abs(yaw_difference) > radians(8):  # Adjust yaw if the difference is significant
                    vel_msg.angular.z = 0.5 if yaw_difference > 0 else -0.5
                    self.state = 'TURN_TO_TARGET'

        self.vel_publisher.publish(vel_msg)
        return False

    def calculate_distance(self, current_position, target_position):
        return sqrt((target_position.x - current_position.x) ** 2 + (target_position.y - current_position.y) ** 2)

    def get_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def obstacle_on_right(self, ranges):
        # Check if there's an obstacle on the right side
        num_readings = len(ranges)
        right_ranges = ranges[:num_readings // 2]
        min_right_range = min(right_ranges)

        return min_right_range < 1.2

def main(args=None):
    rclpy.init(args=args)
    ground_robot_controller = GroundRobotController()

    rclpy.spin(ground_robot_controller)

    ground_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
