import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleNode(Node):
    def __init__(self):
        super().__init__('circle_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        v = 1.0   # linear speed m/s
        r = 1.0   # radius m
        w = v / r # angular speed rad/s
        self.twist.linear.x = float(v)
        self.twist.angular.z = float(w)
        self.get_logger().info(f'Started circle_node -> v={v}, r={r}, w={w}')

    def timer_callback(self):
        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
