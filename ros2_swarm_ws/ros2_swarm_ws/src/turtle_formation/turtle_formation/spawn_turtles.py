#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class Spawner(Node):
    def __init__(self):
        super().__init__('spawner')
        self.cli = self.create_client(Spawn, 'spawn')
        # wait for the service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        # 1) spawn the prey
        self._spawn(10.0, 5.5, math.pi, 'prey')
        # 2) spawn 5 predators on the left
        n = 5
        y_min, y_max = 1.0, 10.0
        spacing = (y_max - y_min)/(n-1)
        for i in range(1, n+1):
            y = y_min + (i-1)*spacing
            self._spawn(1.0, y, 0.0, f'turtle{i}')
        self.get_logger().info('All turtles spawned, shutting down spawner.')
        rclpy.shutdown()

    def _spawn(self, x, y, theta, name):
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.name = name
        self.cli.call_async(req)

def main():
    rclpy.init()
    node = Spawner()
    rclpy.spin(node)
    # cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
