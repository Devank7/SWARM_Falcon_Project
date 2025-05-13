#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def normalize_angle(a: float) -> float:
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class PredatorSwarm(Node):
    def __init__(self):
        super().__init__('predator_swarm')
        # --- declare parameters ---
        self.declare_parameter('index',      1)
        self.declare_parameter('num_agents', 5)
        self.declare_parameter('detection_radius', 4.0)
        self.declare_parameter('capture_radius',   0.5)
        self.declare_parameter('encircle_radius',  3.0)
        self.declare_parameter('patrol_speed',     1.0)
        self.declare_parameter('return_speed',     1.5)

        # --- read them ---
        self.index      = self.get_parameter('index').value
        self.num_agents = self.get_parameter('num_agents').value
        p              = self.get_parameter
        self.detect_r  = p('detection_radius').value
        self.cap_r     = p('capture_radius').value
        self.enc_r     = p('encircle_radius').value
        self.patrol_v  = p('patrol_speed').value
        self.return_v  = p('return_speed').value

        # --- turtle pose subscription ---
        self.current = None
        self.initial = None
        self.create_subscription(
            Pose, f"/turtle{self.index}/pose",
            self._on_pose, 10)

        # --- prey subscription ---
        self.prey = None
        self.create_subscription(
            Pose, "/prey/pose",
            self._on_prey, 10)

        # --- state broadcast ---
        self.state = 'PATROL'
        self.state_pub = self.create_publisher(
            String, f"/turtle{self.index}/state", 10)

        # --- neighbor state subscriptions ---
        self.neighbor_states = {}
        for i in range(1, self.num_agents+1):
            if i == self.index: continue
            self.create_subscription(
                String,
                f"/turtle{i}/state",
                lambda msg, i=i: self.neighbor_states.__setitem__(i, msg.data),
                10)

        # --- velocity publisher ---
        self.cmd_pub = self.create_publisher(
            Twist, f"/turtle{self.index}/cmd_vel", 10)

        # --- start control loop ---
        self._broadcast_state()
        self.patrol_goal = None
        self.create_timer(0.1, self._control_loop)

    def _on_pose(self, msg: Pose):
        if self.current is None:
            self.initial = (msg.x, msg.y)
        self.current = msg

    def _on_prey(self, msg: Pose):
        self.prey = msg

    def _broadcast_state(self):
        m = String()
        m.data = self.state
        self.state_pub.publish(m)

    def _go_to(self, tx, ty, speed) -> float:
        dx, dy = tx - self.current.x, ty - self.current.y
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        err = normalize_angle(yaw - self.current.theta)
        cmd = Twist()
        cmd.linear.x  = speed * dist
        cmd.angular.z = 4.0 * err
        self.cmd_pub.publish(cmd)
        return dist

    def _control_loop(self):
        if not self.current or not self.prey:
            return

        d_prey = math.hypot(
            self.prey.x - self.current.x,
            self.prey.y - self.current.y)

        # --- transitions ---
        if self.state == 'PATROL':
            if (d_prey < self.detect_r or
                any(s == 'ENCIRCLE' for s in self.neighbor_states.values())):
                self.state = 'ENCIRCLE'
                self._broadcast_state()

        elif self.state == 'ENCIRCLE':
            if d_prey < self.cap_r:
                self.state = 'RETURN'
                self._broadcast_state()

        elif self.state == 'RETURN':
            dx = self.current.x - self.initial[0]
            dy = self.current.y - self.initial[1]
            if math.hypot(dx, dy) < 0.2:
                self.state = 'PATROL'
                self.neighbor_states.clear()
                self.patrol_goal = None
                self._broadcast_state()

        # --- behaviors ---
        if self.state == 'PATROL':
            if (self.patrol_goal is None or
                self._go_to(*self.patrol_goal, self.patrol_v) < 0.2):
                self.patrol_goal = (random.uniform(1.0,10.0),
                                    random.uniform(1.0,10.0))

        elif self.state == 'ENCIRCLE':
            members = [i for i,s in self.neighbor_states.items() if s=='ENCIRCLE']
            members.append(self.index)
            members = sorted(members)
            idx = members.index(self.index)
            N   = len(members)
            base = math.atan2(
                self.prey.y - self.current.y,
                self.prey.x - self.current.x)
            theta = base + 2*math.pi*idx/N
            tx = self.prey.x + self.enc_r*math.cos(theta)
            ty = self.prey.y + self.enc_r*math.sin(theta)
            self._go_to(tx,ty,self.patrol_v)

        elif self.state == 'RETURN':
            self._go_to(*self.initial, self.return_v)

def main():
    rclpy.init()
    node = PredatorSwarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
