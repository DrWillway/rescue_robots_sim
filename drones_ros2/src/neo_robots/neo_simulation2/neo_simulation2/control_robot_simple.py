import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, radians, pi
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = time.time()

    def update(self, error):
        current_time = time.time()
        delta_time = current_time - self.previous_time
        if delta_time == 0:
            return 0
        
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.previous_error = error
        self.previous_time = current_time

        return output

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
        
        # Initialize subscribers and publishers with the namespace
        self.target_pose_subscriber = self.create_subscription(PoseStamped, target_pose_topic, self.target_pose_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.target_pose = None
        self.current_pose = None
        self.state = 'IDLE'

        # Initialize PID controllers
        self.linear_pid = PIDController(1.0, 0.0, 0.1)  # Tune these values
        self.angular_pid = PIDController(1.0, 0.0, 0.1)  # Tune these values

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose
        self.state = 'MOVE_TO_POSITION'
        # Extract the target yaw from the target pose orientation
        target_yaw = self.get_yaw(self.target_pose.orientation)
        self.move_to_position(self.target_pose.position, target_yaw)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.state != 'IDLE':
            target_yaw = self.get_yaw(self.target_pose.orientation)
            self.move_to_position(self.target_pose.position, target_yaw)

    def move_to_position(self, target, target_yaw):
        if self.current_pose is None:
            return False

        current_position = self.current_pose.position
        distance_to_target = self.calculate_distance(current_position, target)

        vel_msg = Twist()

        if distance_to_target < 0.5:  # Threshold distance to consider the target reached
            current_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_difference = self.normalize_angle(target_yaw - current_yaw)

            if abs(yaw_difference) > radians(5):  # Threshold angle to consider the orientation reached
                angular_correction = self.angular_pid.update(yaw_difference)
                vel_msg.angular.z = angular_correction
                self.state = 'ADJUST_YAW'
            else:
                self.state = 'IDLE'
                self.vel_publisher.publish(Twist())  # Stop the robot
                return True
        else:
            angle_to_target = atan2(target.y - current_position.y, target.x - current_position.x)
            current_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_difference = self.normalize_angle(angle_to_target - current_yaw)

            if abs(yaw_difference) > radians(5):  # Adjust yaw if the difference is significant
                angular_correction = self.angular_pid.update(yaw_difference)
                vel_msg.angular.z = angular_correction
                self.state = 'TURN_TO_TARGET'
            else:
                linear_correction = self.linear_pid.update(distance_to_target)
                vel_msg.linear.x = linear_correction
                self.state = 'MOVE_TO_TARGET'

        self.vel_publisher.publish(vel_msg)
        return False

    def calculate_distance(self, current_position, target_position):
        return sqrt((target_position.x - current_position.x) ** 2 + (target_position.y - current_position.y) ** 2)

    def get_yaw(self, orientation):
        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    ground_robot_controller = GroundRobotController()

    rclpy.spin(ground_robot_controller)

    ground_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
