import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import radians, sqrt, pow, atan2, pi

class DroneControl(Node):

    def __init__(self):
        super().__init__(f"drone_control") # need namespace here
        
        # Declare and read parameters
        self.declare_parameter('namespace', 'drone')
        self.declare_parameter('x_start', 0.0)
        self.declare_parameter('y_start', 0.0)
        self.declare_parameter('width', 3.5)
        self.declare_parameter('depth', 14.0)
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        x_start = self.get_parameter('x_start').get_parameter_value().double_value
        y_start = self.get_parameter('y_start').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().double_value
        depth = self.get_parameter('depth').get_parameter_value().double_value


        self.namespace = namespace
        self.takeoff_publisher = self.create_publisher(Empty, f"/{namespace}/takeoff", 10)
        self.vel_publisher = self.create_publisher(Twist, f"/{namespace}/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            f"/{namespace}/odom",
            self.odom_callback,
            10
        )
        self.timer_period = 0.1  # seconds
        self.create_timer(self.timer_period, self.main_loop)

        self.odom = Odometry()
        self.state = "start"
        self.target_explore_distance = 3.0
        self.current_distance = 0.0
        self.start_position = None
        self.start_orientation = None
        self.turn_ticks = 0
        self.target_height = 7.0
        self.target_start_position = Point(x=x_start, y=y_start, z=self.target_height)  # Target position to reach before exploring
        self.target_yaw = radians(0)  # Target orientation (yaw) to reach

        self.waypoints = []
        num_points = 3
        self.create_zigzag(x_start, y_start, self.target_height, width, depth, num_points)
        self.current_waypoint_index = 0

    # create zigzag movement pattern
    def create_zigzag(self, start_x, start_y, target_height, width, depth, num_points=10):
        # Calculate the amplitude based on the total distance and number of points
        amplitude = abs(depth) / (num_points - 1)
        
        # Generate y coordinates
        y_coords = [start_y + i * amplitude for i in range(num_points)]
        
        # Initialize x coordinates with the starting position
        x_coords = [start_x]

        # Start with right movement
        direction = 1
        
        # Generate x coordinates to form a zigzag pattern
        for i in range(1, num_points):
            next_x = x_coords[-1] + direction * width
            x_coords.append(next_x)
            direction *= -1  # Switch direction
                
        x_coords.extend(x_coords[::-1])
        y_coords.extend(y_coords[::-1])

        # Append the coordinates to self.waypoints as Point objects
        self.waypoints = [Point(x=x, y=y, z=target_height) for x, y in zip(x_coords, y_coords)]


    def odom_callback(self, msg):
        self.odom = msg
        # self.get_logger().info(f"Received odometry data: {msg}")

    def main_loop(self):
        if self.state == "start":
            if self.odom.pose.pose.position.z < 0.5:
                self.takeoff_publisher.publish(Empty())
                self.get_logger().info(f"Taking off: {self.namespace}")
            elif self.odom.pose.pose.position.z < self.target_height:
                vel_msg = Twist()
                vel_msg.linear.z = 0.8
                self.vel_publisher.publish(vel_msg)
                self.get_logger().info(f"{self.namespace} state: Ascending to position z: {self.target_height}, current z: {self.odom.pose.pose.position.z}")
            else:
                self.get_logger().info(f"{self.namespace} state: Hovering at position z: {self.target_height}, current z: {self.odom.pose.pose.position.z}")
                self.state = "explore"
                self.start_position = self.odom.pose.pose.position
        elif self.state == "explore":
            if self.current_waypoint_index < len(self.waypoints):
                target_waypoint = self.waypoints[self.current_waypoint_index]
                target_reached = self.move_to_position(target_waypoint, self.target_yaw)
                if target_reached:
                    self.current_waypoint_index += 1
                    self.get_logger().info(f"{self.namespace} Reached waypoint {self.current_waypoint_index}, moving to next waypoint")
            else:
                self.get_logger().info(f"{self.namespace} Completed exploration")

    def move_to_position(self, target, target_yaw):
        current_position = self.odom.pose.pose.position
        distance_to_target = self.calculate_distance(current_position, target)

        if distance_to_target < 0.5:  # Threshold distance to consider the target reached
            current_yaw = self.get_yaw(self.odom.pose.pose.orientation)
            yaw_difference = self.normalize_angle(target_yaw - current_yaw)

            if abs(yaw_difference) > radians(360):  # Threshold angle to consider the orientation reached
                vel_msg = Twist()
                vel_msg.angular.z = 0.8 if yaw_difference > 0 else -0.8
                self.vel_publisher.publish(vel_msg)
            else:
                vel_msg = Twist()
                self.vel_publisher.publish(vel_msg)  # Stop the drone
                return True
        else:
            angle_to_target = atan2(target.y - current_position.y, target.x - current_position.x)
            current_yaw = self.get_yaw(self.odom.pose.pose.orientation)
            yaw_difference = self.normalize_angle(angle_to_target - current_yaw)

            vel_msg = Twist()
            if abs(yaw_difference) > radians(5):  # Adjust yaw if the difference is significant
                vel_msg.angular.z = 0.8 if yaw_difference > 0 else -0.8
            else:
                vel_msg.linear.x = 0.8

            self.vel_publisher.publish(vel_msg)

        return False

    def calculate_distance(self, start, end):
        return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2))

    def get_yaw(self, orientation):
        q = orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > radians(180):
            angle -= radians(360)
        while angle < -radians(180):
            angle += radians(360)
        return angle

def main(args=None):
    rclpy.init(args=args)

    drone_control = DroneControl()

    rclpy.spin(drone_control)

    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
