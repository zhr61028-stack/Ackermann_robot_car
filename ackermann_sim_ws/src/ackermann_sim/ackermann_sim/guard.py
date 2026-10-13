"""Simulation-only Twist gate. No MCU protocol or vehicle driver."""
import json
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .assets import validate


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        path = self.declare_parameter('config_path', '').value
        config = yaml.safe_load(Path(path).read_text())
        self.radius = validate(config)
        self.vehicle = config['vehicle']; self.control = config['control']
        self.allow_reverse = self.declare_parameter('allow_reverse', True).value
        self.last_rx = None; self.command = Twist(); self.invalid = False
        self.pub = self.create_publisher(Twist, '/sim/cmd_vel', 1)
        self.status = self.create_publisher(String, '/sim/watchdog', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.receive, QoSProfile(depth=1))
        # A paused simulation must not pause the receive-time watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.control['timer_period_s'], self.update, clock=self.steady_clock)

    def receive(self, msg):
        self.last_rx = time.monotonic()
        values = [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        self.invalid = not all(math.isfinite(v) for v in values)
        self.invalid |= any(abs(v) > 1e-9 for v in [msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y])
        self.command = msg

    def update(self):
        now = time.monotonic()
        age = None if self.last_rx is None else now - self.last_rx
        expired = age is None or age >= self.control['timeout_s']
        out = Twist(); reason = 'ok'
        if expired:
            reason = 'no_command' if age is None else 'timeout'
        elif self.invalid:
            reason = 'invalid_command'
        else:
            speed = max(-self.vehicle['max_speed_mps'], min(self.vehicle['max_speed_mps'], self.command.linear.x))
            if speed < 0 and not self.allow_reverse:
                reason = 'reverse_rejected'
            elif abs(speed) < self.control['zero_speed_threshold_mps']:
                reason = 'pure_rotation_rejected' if abs(self.command.angular.z)>1e-9 else 'stop_command'
            else:
                out.linear.x = speed
                bound = abs(speed) / self.radius
                out.angular.z = max(-bound, min(bound, self.command.angular.z))
                if speed != self.command.linear.x or out.angular.z != self.command.angular.z:
                    reason = 'limited'
        self.pub.publish(out)
        self.status.publish(String(data=json.dumps(dict(monotonic_s=now,last_rx_monotonic_s=self.last_rx,
            age_s=age,ros_ns=self.get_clock().now().nanoseconds,expired=expired,reason=reason,
            safe_speed=out.linear.x,safe_yaw_rate=out.angular.z))))


def main():
    rclpy.init(); node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
