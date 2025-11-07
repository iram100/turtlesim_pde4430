import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import time

class SpawnMulti(Node):
    def __init__(self):
        super().__init__('spawn_multi')
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting...')
        self.turtles = []
        # spawn 3 extra turtles
        self.spawn_turtle(5.5, 5.5, 0.0, 'turtle2')
        time.sleep(0.2)
        self.spawn_turtle(2.0, 8.0, 0.0, 'turtle3')
        time.sleep(0.2)
        self.spawn_turtle(9.0, 2.0, 0.0, 'turtle4')
        # create publishers for their cmd_vel topics
        self.pubs = []
        for name in ['turtle2','turtle3','turtle4']:
            self.pubs.append(self.create_publisher(Twist, f'/{name}/cmd_vel', 10))
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.t = 0.0

    def spawn_turtle(self, x, y, theta, name):
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.name = name
        fut = self.cli.call_async(req)
        # don't block long
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        if fut.done():
            res = fut.result()
            self.get_logger().info(f'spawned {res.name}')

    def timer_cb(self):
        from math import sin, cos
        msg_list = []
        for i, pub in enumerate(self.pubs):
            tw = Twist()
            tw.linear.x = 1.0 + 0.2 * i
            tw.angular.z = 0.5 * (i+1) * 0.2
            pub.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    node = SpawnMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
