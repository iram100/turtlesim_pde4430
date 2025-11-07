import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StraightLine(Node):
    def __init__(self):
        super().__init__('straight_line')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        self.twist.linear.x = 1.0
        self.get_logger().info('Started straight_line -> publishing to /turtle1/cmd_vel at 1.0 m/s')

    def timer_callback(self):
        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = StraightLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
