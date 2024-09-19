import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        
        # Declare and read parameters
        self.declare_parameter('namespace', 'default_value')
        namespace = self.get_parameter('namespace').get_parameter_value().string_value

        self.declare_parameter('x_start', 10.0)
        x_start = self.get_parameter('x_start').get_parameter_value().double_value
        
        self.get_logger().info(f'namespace value: {namespace}')
        self.get_logger().info(f'x_start value: {x_start}')

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
