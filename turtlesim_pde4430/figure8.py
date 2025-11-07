import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class Figure8(Node):
    def __init__(self):
        super().__init__('figure8')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.t = 0.0
        self.dt = 0.05

    def timer_callback(self):
        # parametric figure-8: x = A * sin(omega * t), y = B * sin(omega * t) * cos(omega * t)
        A = 1.5
        omega = 0.5
        vx = A * omega * math.cos(omega * self.t)
        vy = A * omega * (math.cos(2 * omega * self.t) - math.sin(2 * omega * self.t)) * 0.5
        # approximate conversion to cmd_vel: linear = sqrt(vx^2+vy^2), angular = vy/vx (naive)
        linear = math.hypot(vx, vy)
        angular = 0.0
        if abs(vx) > 1e-6:
            angular = vy / vx
        msg = Twist()
        msg.linear.x = max(min(linear, 2.0), -2.0)
        msg.angular.z = angular
        self.pub.publish(msg)
        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = Figure8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
