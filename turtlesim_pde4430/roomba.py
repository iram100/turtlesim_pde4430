import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math

class Roomba(Node):
    def __init__(self):
        super().__init__('roomba')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pose = None
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.state = 'forward'
        self.forward_speed = 1.0
        self.turn_speed = 1.0

    def pose_cb(self, msg):
        self.pose = msg

    def timer_cb(self):
        if self.pose is None:
            return
        # turtlesim window is 11x11 (from 0 to 11); boundaries at ~0 and ~11
        x, y = self.pose.x, self.pose.y
        hit = x < 0.5 or x > 10.5 or y < 0.5 or y > 10.5
        twist = Twist()
        if hit:
            # rotate by random angle then go forward
            angle = random.uniform(-math.pi, math.pi)
            twist.angular.z = angle
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Roomba()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
