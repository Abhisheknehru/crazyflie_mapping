"""Watch the four horizontal Multi-ranger topics and flag a stuck sensor."""
import math
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RangeMonitor(Node):
    def __init__(self):
        super().__init__('range_monitor')
        self.declare_parameter('window', 20)
        self.declare_parameter('static_threshold', 0.005)
        window = int(self.get_parameter('window').value)
        self.threshold = float(self.get_parameter('static_threshold').value)
        if window < 2:
            raise ValueError('window must be at least 2 samples')
        if not (math.isfinite(self.threshold) and self.threshold > 0):
            raise ValueError('static_threshold must be a positive number')
        self.samples = {face: deque(maxlen=window)
                        for face in ('front', 'left', 'back', 'right')}
        self.last_seen = {}
        for face in self.samples:
            self.create_subscription(
                Range, '/crazyflie/range_' + face, self.on_range(face),
                qos_profile_sensor_data)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'Watching front/left/back/right ranges; window={window} samples, '
            f'static_threshold={self.threshold} m')

    def on_range(self, face):
        def callback(msg):
            self.samples[face].append(msg.range)
            self.last_seen[face] = seconds(self.get_clock().now().to_msg())
        return callback

    def report(self):
        now = seconds(self.get_clock().now().to_msg())
        lines = []
        for face, values in self.samples.items():
            last = self.last_seen.get(face)
            if last is None or now - last > 1.0:
                lines.append(f'{face:>5}: SILENT (no messages in last 1s)')
                continue
            spread = max(values) - min(values)
            flag = ' STATIC' if len(values) == values.maxlen and spread < self.threshold else ''
            lines.append(
                f'{face:>5}: latest={values[-1]:.3f}m  '
                f'spread={spread:.4f}m over {len(values)} samples{flag}')
        self.get_logger().info('\n' + '\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RangeMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
