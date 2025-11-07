import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import argparse

class GoTo(Node):
    def __init__(self, goal_x, goal_y, linear_k=1.0, angular_k=4.0):
        super().__init__('goto')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.pose = None
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.timer = self.create_timer(0.1, self.control_cb)
        self.lin_k = linear_k
        self.ang_k = angular_k

    def pose_cb(self, msg):
        self.pose = msg

    def control_cb(self):
        if self.pose is None:
            return
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        diff_angle = angle_to_goal - self.pose.theta
        # normalize
        diff_angle = math.atan2(math.sin(diff_angle), math.cos(diff_angle))
        tw = Twist()
        if dist > 0.1:
            tw.linear.x = self.lin_k * dist
            tw.angular.z = self.ang_k * diff_angle
        else:
            tw.linear.x = 0.0
            tw.angular.z = 0.0
        self.pub.publish(tw)

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=5.5)
    parser.add_argument('--y', type=float, default=5.5)
    parser.add_argument('--linear', type=float, default=1.0)
    parser.add_argument('--angular', type=float, default=4.0)
    parsed, _ = parser.parse_known_args()
    rclpy.init()
    node = GoTo(parsed.x, parsed.y, parsed.linear, parsed.angular)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
