"""Publish exploration proposals only; this node has no flight connection."""
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

from test_runs.reactive_policy import ReactivePolicy


class ReactiveExplorer(Node):
    def __init__(self):
        super().__init__('reactive_explorer')
        defaults = {'speed': 0.10, 'clearance': 0.30, 'wall_distance': 0.45,
                    'opening': 0.75, 'sensor_timeout': 0.5,
                    'max_duration': 120.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: float(self.get_parameter(name).value) for name in defaults}
        self.policy = ReactivePolicy(**{k: values[k] for k in
                                       ('speed', 'clearance', 'wall_distance', 'opening')})
        self.timeout = values['sensor_timeout']
        self.duration = values['max_duration']
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.duration)):
            raise ValueError('Timeout and duration must be finite and positive')
        self.ranges = {}
        self.received = {}
        self.started = None
        self.last_state = None
        self.commands = self.create_publisher(TwistStamped, '/exploration/proposed_velocity', 10)
        self.states = self.create_publisher(String, '/exploration/state', 10)
        for face in ('front', 'left', 'back', 'right'):
            self.create_subscription(Range, '/crazyflie/range_' + face,
                                     self.callback(face), qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Proposal-only mode: no takeoff or flight commands. Heading must stay fixed.')

    def callback(self, face):
        def receive(msg):
            value = float(msg.range)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            valid = (math.isfinite(value) and 0 < value and
                     msg.min_range <= value <= msg.max_range and
                     stamp > 0 and -0.1 <= age <= self.timeout)
            # Unknown/out-of-range data is not assumed to mean free space.
            self.ranges[face] = value if valid else math.nan
            self.received[face] = time.monotonic()
        return receive

    def tick(self):
        now = time.monotonic()
        vx = vy = 0.0
        if len(self.received) != 4 or any(now - t > self.timeout for t in self.received.values()):
            state = 'WAITING_FOR_FRESH_RANGES'
        elif self.started is not None and now - self.started >= self.duration:
            state = 'DURATION_STOP'
        else:
            vx, vy, state = self.policy.step(self.ranges)
            if self.started is None and state not in ('INVALID_RANGE', 'CLEARANCE_STOP'):
                self.started = now
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = vx
        command.twist.linear.y = vy
        self.commands.publish(command)
        status = String()
        status.data = state
        self.states.publish(status)
        if state != self.last_state:
            #self.get_logger().info(state)
            self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
