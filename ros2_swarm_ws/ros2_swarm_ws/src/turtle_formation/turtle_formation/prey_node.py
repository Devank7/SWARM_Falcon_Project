#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Prey(Node):
    def __init__(self):
        super().__init__('prey_node')
        self.declare_parameter('speed', 1.5)
        self.speed = self.get_parameter('speed').value

        # track own pose
        self.pose = None
        self.create_subscription(
            Pose, '/prey/pose', self._on_pose, 10)

        # cmd_vel publisher
        self.pub = self.create_publisher(
            Twist, '/prey/cmd_vel', 10)

        # wander timer
        self.create_timer(0.2, self._move)

    def _on_pose(self, msg: Pose):
        self.pose = msg

    def _move(self):
        if not self.pose:
            return
        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = random.uniform(-1.0, 1.0)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = Prey()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
