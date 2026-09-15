"""One telemetry-only Crazyradio connection feeding the ROS exploration stack."""
from collections import deque
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range


def range_metres(raw, minimum=0.02, maximum=4.0):
    """Finite saturated metres; unknown/error readings use the stop distance.

    Multi-ranger firmware uses 32767 mm for rejected measurements. Never
    reinterpret that sentinel as a clear path. 65535 is also treated as invalid.
    """
    if not (math.isfinite(minimum) and math.isfinite(maximum) and
            0 < minimum < maximum <= 4.0):
        raise ValueError('Require 0 < ToF minimum < maximum <= 4.0 metres')
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return minimum
    if not math.isfinite(value) or value <= 0 or value >= 32767:
        return minimum
    return max(minimum, min(maximum, value / 1000.0))


class RadioBridge(Node):
    def __init__(self):
        super().__init__('exploration_radio_bridge')
        self.declare_parameter('uri', 'radio://0/80/2M/E7E7E7E7E7')
        self.declare_parameter('log_period_ms', 100)
        self.declare_parameter('tof_min_range', 0.02)
        self.declare_parameter('tof_max_range', 4.0)
        self.tof_min = float(self.get_parameter('tof_min_range').value)
        self.tof_max = float(self.get_parameter('tof_max_range').value)
        range_metres(1000, self.tof_min, self.tof_max)
        self.declare_parameter('connect_timeout', 20.0)
        self.declare_parameter('telemetry_timeout', 3.0)
        self.uri = str(self.get_parameter('uri').value)
        self.period = int(self.get_parameter('log_period_ms').value)
        self.connect_timeout = float(self.get_parameter('connect_timeout').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        if not self.uri.startswith('radio://'):
            raise ValueError('uri must start with radio://')
        if self.period < 10 or self.period > 200 or self.period % 10:
            raise ValueError('log_period_ms must be a multiple of 10 between 10 and 200')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.connect_timeout, self.telemetry_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/crazyflie/pose', 10)
        self.range_pubs = {face: self.create_publisher(
            Range, '/crazyflie/range_' + face, 10)
            for face in ('front', 'left', 'back', 'right', 'up', 'down')}
        self.events = deque(maxlen=100)
        self.logs = []
        self.origin = None
        self.last_data = {}
        self.connected_at = None
        self.reset_phase = 'CONNECTING'
        self.reset_deadline = None
        self.ready = False
        self.variance_history = deque(maxlen=10)
        self.error = None
        self.closing = False
        self.started = time.monotonic()
        cflib.crtp.init_drivers()
        self.cf = Crazyflie()
        self.cf.fully_connected.add_callback(self.on_connected)
        self.cf.connection_failed.add_callback(self.on_failed)
        self.cf.connection_lost.add_callback(self.on_failed)
        self.cf.disconnected.add_callback(self.on_disconnected)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'Connecting telemetry to {self.uri}')
        self.cf.open_link(self.uri)

    def on_failed(self, uri, message):
        self.error = f'{uri}: {message}'

    def on_disconnected(self, uri):
        if not self.closing:
            self.error = f'Radio disconnected: {uri}'

    def on_connected(self, uri):
        self.connected_at = time.monotonic()
        self.reset_phase = 'SELECT_ESTIMATOR'

    def start_logs(self):
        specifications = {
            'pose': [('stateEstimate.' + name, 'float') for name in ('x', 'y', 'z', 'yaw')],
            'ranges': [('range.' + name, 'uint16_t') for name in
                       ('front', 'left', 'back', 'right', 'up', 'zrange')],
            'variance': [('kalman.varP' + axis, 'float') for axis in ('X', 'Y', 'Z')],
        }
        for name, variables in specifications.items():
            config = LogConfig(name='exploration_' + name,
                               period_in_ms=500 if name == 'variance' else self.period)
            for variable, kind in variables:
                config.add_variable(variable, kind)
            self.cf.log.add_config(config)
            config.data_received_cb.add_callback(self.receive(name))
            config.error_cb.add_callback(self.log_error)
            self.logs.append(config)
            config.start()

    def advance_reset(self, now):
        if self.reset_phase == 'SELECT_ESTIMATOR':
            self.get_logger().info('Resetting Kalman estimator; keep the drone stationary on the ground.')
            self.cf.param.set_value('stabilizer.estimator', '2')
            self.reset_deadline = now + 0.5
            self.reset_phase = 'RESET_HIGH'
        elif self.reset_phase == 'RESET_HIGH' and now >= self.reset_deadline:
            self.cf.param.set_value('kalman.resetEstimation', '1')
            self.reset_deadline = now + 0.1
            self.reset_phase = 'RESET_LOW'
        elif self.reset_phase == 'RESET_LOW' and now >= self.reset_deadline:
            self.cf.param.set_value('kalman.resetEstimation', '0')
            self.reset_phase = 'SETTLING'
            self.reset_deadline = now + 30.0
            self.start_logs()
        elif self.reset_phase == 'SETTLING' and now >= self.reset_deadline:
            raise RuntimeError('Estimator did not settle within 30 seconds')

    def log_error(self, config, message):
        self.error = f'{config.name}: {message}'

    def receive(self, kind):
        def callback(timestamp, data, config):
            # Stamp on reception; do not mix the firmware boot clock with ROS time.
            self.events.append((kind, dict(data), self.get_clock().now().to_msg(), time.monotonic()))
        return callback

    def tick(self):
        now = time.monotonic()
        if self.error:
            raise RuntimeError(self.error)
        if self.connected_at is None and now - self.started > self.connect_timeout:
            raise RuntimeError('Crazyradio connection timed out')
        self.advance_reset(now)
        while self.events:
            kind, data, stamp, received = self.events.popleft()
            self.last_data[kind] = received
            if now - received > self.telemetry_timeout:
                continue
            if kind == 'variance':
                values = tuple(float(data['kalman.varP' + axis]) for axis in ('X', 'Y', 'Z'))
                if not all(math.isfinite(v) and v >= 0 for v in values):
                    self.variance_history.clear()
                    continue
                self.variance_history.append(values)
                if not self.ready and len(self.variance_history) == 10 and all(
                        max(v[i] for v in self.variance_history) -
                        min(v[i] for v in self.variance_history) < 0.001 for i in range(3)):
                    self.origin = None
                    self.ready = True
                    self.reset_phase = 'READY'
                    # Drop queued samples so the origin is a fresh post-settling pose.
                    self.events.clear()
                    self.get_logger().info('Estimator settled; starting a fresh pose origin.')
                continue
            if not self.ready:
                continue
            if kind == 'pose':
                self.publish_pose(data, stamp)
            else:
                for face, publisher in self.range_pubs.items():
                    msg = Range()
                    msg.header.stamp = stamp
                    msg.header.frame_id = 'range_' + face
                    msg.radiation_type = Range.INFRARED
                    msg.field_of_view = math.radians(27)
                    msg.min_range = self.tof_min
                    msg.max_range = self.tof_max
                    key = 'zrange' if face == 'down' else face
                    msg.range = range_metres(data['range.' + key], self.tof_min, self.tof_max)
                    publisher.publish(msg)
        if self.ready:
            for kind in ('pose', 'ranges'):
                if now - self.last_data.get(kind, self.connected_at) > self.telemetry_timeout:
                    raise RuntimeError(f'{kind} telemetry timed out; check decks and firmware logs')

    def publish_pose(self, data, stamp):
        x, y, z, yaw = [float(data['stateEstimate.' + k]) for k in ('x', 'y', 'z', 'yaw')]
        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            raise RuntimeError('Non-finite onboard pose')
        if self.origin is None:
            self.origin = (x, y, z, math.radians(yaw))
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[3]), math.sin(self.origin[3])
        msg.pose.pose.position.x = c * dx + s * dy
        msg.pose.pose.position.y = -s * dx + c * dy
        msg.pose.pose.position.z = z - self.origin[2]
        # Rotate both translation and yaw into the new start-heading frame.
        yaw_relative = math.radians(yaw) - self.origin[3]
        msg.pose.pose.orientation.z = math.sin(yaw_relative / 2)
        msg.pose.pose.orientation.w = math.cos(yaw_relative / 2)
        self.odom_pub.publish(msg)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

    def destroy_node(self):
        self.closing = True
        self.cf.close_link()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RadioBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
