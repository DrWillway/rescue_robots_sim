import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, radians, pi

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

        self.station_pose = (8.0, 8.0)
        if namespace == 'robot2':
            self.station_pose = (-8.0, 8.0)

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose
        self.state = 'MOVE_TO_TARGET'
        # Extract the target yaw from the target pose orientation
        # target_yaw = self.get_yaw(self.target_pose.orientation)
        # self.move_to_position(self.target_pose.position, target_yaw)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.state != 'IDLE':
            target_yaw = self.get_yaw(self.target_pose.orientation)
            if self.move_to_position(self.target_pose.position, target_yaw):
                self.target_pose.position.x = self.station_pose[0]
                self.target_pose.position.y = self.station_pose[1]

    def move_to_position(self, target, target_yaw):
        if self.current_pose is None:
            return False

        current_position = self.current_pose.position
        distance_to_target = self.calculate_distance(current_position, target)
        vel_msg = Twist()

        if distance_to_target < 1.0:  # Threshold distance to consider the target reached
            current_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_difference = self.normalize_angle(target_yaw - current_yaw)

            self.vel_publisher.publish(Twist())  # Stop the robot
            return True
        else:
            angle_to_target = atan2(target.y - current_position.y, target.x - current_position.x)
            current_yaw = self.get_yaw(self.current_pose.orientation)
            yaw_difference = self.normalize_angle(angle_to_target - current_yaw)

            if abs(yaw_difference) > radians(5):  # Adjust yaw if the difference is significant
                vel_msg.angular.z = 0.5 if yaw_difference > 0 else -0.5
                self.state = 'TURN_TO_TARGET'
            else:
                vel_msg.linear.x = 1.5 if distance_to_target > 1.0 else 0.3
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
