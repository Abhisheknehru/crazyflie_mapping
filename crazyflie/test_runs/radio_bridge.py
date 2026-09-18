"""One telemetry-only Crazyradio connection feeding the ROS exploration stack."""
from collections import deque
import math
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Range

from test_runs.flight_frame import path_is_clear, to_estimator_frame
from test_runs.map_loader import load_map, pick_latest_map


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


def align_planar_pose(local_pose, alignment):
    """Transform local x/y/yaw into a user-selected map pose."""
    x, y, yaw = local_pose
    if alignment is None:
        return x, y, yaw
    local_x, local_y, local_yaw, map_x, map_y, map_yaw = alignment
    turn = map_yaw - local_yaw
    dx, dy = x - local_x, y - local_y
    c, s = math.cos(turn), math.sin(turn)
    return (map_x + c*dx - s*dy,
            map_y + s*dx + c*dy,
            math.atan2(math.sin(yaw + turn), math.cos(yaw + turn)))


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
        self.declare_parameter('settle_timeout', 30.0)
        self.declare_parameter('require_initial_pose', False)
        self.declare_parameter('rw_cache', str(Path.cwd() / 'cache' / 'crazyflie'))
        self.declare_parameter('flight_height', .3)
        self.declare_parameter('takeoff_duration', 2.0)
        self.declare_parameter('land_duration', 2.0)
        self.declare_parameter('goto_duration', 3.0)
        self.declare_parameter('goal_topic', '/crazyflie/goal_pose')
        self.declare_parameter('maps_dir', 'maps')
        self.declare_parameter('dry_run', True)
        self.settle_timeout = float(self.get_parameter('settle_timeout').value)
        self.require_initial_pose = bool(
            self.get_parameter('require_initial_pose').value)
        self.uri = str(self.get_parameter('uri').value)
        self.period = int(self.get_parameter('log_period_ms').value)
        self.connect_timeout = float(self.get_parameter('connect_timeout').value)
        self.telemetry_timeout = float(self.get_parameter('telemetry_timeout').value)
        self.flight_height = float(self.get_parameter('flight_height').value)
        self.takeoff_duration = float(self.get_parameter('takeoff_duration').value)
        self.land_duration = float(self.get_parameter('land_duration').value)
        self.goto_duration = float(self.get_parameter('goto_duration').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        if not self.uri.startswith('radio://'):
            raise ValueError('uri must start with radio://')
        if self.period < 10 or self.period > 200 or self.period % 10:
            raise ValueError('log_period_ms must be a multiple of 10 between 10 and 200')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.connect_timeout, self.telemetry_timeout, self.settle_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.flight_height, self.takeoff_duration,
                    self.land_duration, self.goto_duration)):
            raise ValueError('Flight height and durations must be finite and positive')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/crazyflie/pose', 10)
        self.range_pubs = {face: self.create_publisher(
            Range, '/crazyflie/range_' + face, 10)
            for face in ('front', 'left', 'back', 'right', 'up', 'down')}
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.on_initial_pose, 10)
        try:
            map_path = pick_latest_map(str(self.get_parameter('maps_dir').value))
            self.localization_map = load_map(map_path)
            self.get_logger().info(f'Loaded {map_path} for goal_pose obstacle checks')
        except FileNotFoundError:
            self.localization_map = None
            self.get_logger().warning(
                'No saved map found; goal_pose commands will be refused until '
                'one exists (build a map first, or fly without goal_pose)')
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value),
            self.on_goal, 10)
        self.flying = False
        self.goto_timer = None
        self.events = deque(maxlen=100)
        self.logs = []
        self.origin = None
        self.latest_local_pose = None
        self.latest_map_pose = None
        self.map_alignment = None
        self.last_data = {}
        self.connected_at = None
        self.reset_phase = 'CONNECTING'
        self.reset_deadline = None
        self.ready = False
        self.variance_history = deque(maxlen=10)
        self.variance_samples = 0
        self.invalid_variance_samples = 0
        self.last_variance_at = None
        self.logs_started_at = None
        self.next_settle_report = 0.0
        self.raw_down = None
        self.error = None
        self.closing = False
        self.started = time.monotonic()
        cflib.crtp.init_drivers()
        cache_dir = Path(str(self.get_parameter('rw_cache').value)).expanduser()
        cache_dir.mkdir(parents=True, exist_ok=True)
        self.cf = Crazyflie(rw_cache=str(cache_dir))
        self.cf.fully_connected.add_callback(self.on_connected)
        self.cf.connection_failed.add_callback(self.on_failed)
        self.cf.connection_lost.add_callback(self.on_failed)
        self.cf.disconnected.add_callback(self.on_disconnected)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'Connecting telemetry to {self.uri}')
        self.cf.open_link(self.uri)

    def on_initial_pose(self, msg):
        if self.latest_local_pose is None:
            self.get_logger().warning(
                'Initial pose ignored: wait for estimator telemetry, then click again')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (msg.header.frame_id != 'map' or
                not all(math.isfinite(value) for value in values) or
                abs(norm - 1.) > .01):
            self.get_logger().warning(
                'Initial pose must be finite and in the map frame')
            return
        map_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1-2*(q.y*q.y + q.z*q.z))
        local_x, local_y, local_yaw = self.latest_local_pose
        self.map_alignment = (local_x, local_y, local_yaw,
                              float(p.x), float(p.y), map_yaw)
        self.get_logger().info(
            f'Live pose aligned: x={p.x:.3f}, y={p.y:.3f}, '
            f'yaw={math.degrees(map_yaw):.1f} deg')

    def on_goal(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (msg.header.frame_id != 'map' or
                not all(math.isfinite(value) for value in values) or
                abs(norm - 1.) > .01):
            self.get_logger().error('goal_pose must be finite and in the map frame')
            return
        if not self.ready or self.origin is None or self.latest_map_pose is None:
            self.get_logger().error('goal_pose refused: estimator not settled yet')
            return
        map_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        map_x, map_y = float(p.x), float(p.y)
        if self.localization_map is None:
            self.get_logger().error(
                'goal_pose refused: no saved map loaded, cannot verify the path is clear')
            return
        if not path_is_clear(self.localization_map, self.latest_map_pose[0],
                              self.latest_map_pose[1], map_x, map_y):
            self.get_logger().error(
                f'goal_pose refused: straight-line path to ({map_x:.2f}, {map_y:.2f}) '
                'crosses a mapped wall')
            return
        raw_x, raw_y, raw_yaw = to_estimator_frame(
            map_x, map_y, map_yaw, self.origin, self.map_alignment)
        raw_z = self.flight_height + self.origin[2]
        if self.dry_run:
            self.get_logger().info(
                f'[dry_run] would fly to raw ({raw_x:.3f}, {raw_y:.3f}, '
                f'{raw_z:.3f}, yaw={math.degrees(raw_yaw):.1f} deg)')
            return
        if not self.flying:
            self.cf.param.set_value('commander.enHighLevel', '1')
            self.cf.high_level_commander.takeoff(raw_z, self.takeoff_duration)
            self.flying = True
            self.get_logger().info(
                f'Taking off to {raw_z:.2f}m, then flying to '
                f'({raw_x:.2f}, {raw_y:.2f})')
            if self.goto_timer is not None:
                self.goto_timer.cancel()

            def after_takeoff():
                self.goto_timer.cancel()
                self.cf.high_level_commander.go_to(
                    raw_x, raw_y, raw_z, raw_yaw, self.goto_duration)
            self.goto_timer = self.create_timer(self.takeoff_duration, after_takeoff)
        else:
            self.cf.high_level_commander.go_to(
                raw_x, raw_y, raw_z, raw_yaw, self.goto_duration)
            self.get_logger().info(f'Flying to ({raw_x:.2f}, {raw_y:.2f})')

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
            self.reset_deadline = now + self.settle_timeout
            self.logs_started_at = now
            self.start_logs()

    def settling_report(self):
        spread = 'not enough valid samples'
        if self.variance_history:
            widths = [max(v[i] for v in self.variance_history) -
                      min(v[i] for v in self.variance_history) for i in range(3)]
            spread = '/'.join(f'{v:.6g}' for v in widths)
        return (f'variance samples={self.variance_samples}, '
                f'invalid={self.invalid_variance_samples}, '
                f'window={len(self.variance_history)}/10, '
                f'X/Y/Z spread={spread} (each must be <0.001), '
                f'raw down ToF={self.raw_down} mm')

    def check_startup(self, now):
        if self.reset_phase != 'SETTLING':
            return
        for kind in ('pose', 'ranges', 'variance'):
            if now - self.last_data.get(kind, self.logs_started_at) > self.telemetry_timeout:
                raise RuntimeError(f'Missing/stale {kind} telemetry during startup; '
                                   'check radio, decks and firmware log support. ' +
                                   self.settling_report())
        if now >= self.reset_deadline:
            raise RuntimeError(f'Estimator did not settle within {self.settle_timeout:g} seconds. ' +
                               self.settling_report())
        if now >= self.next_settle_report:
            self.get_logger().info('Waiting for estimator: ' + self.settling_report())
            self.next_settle_report = now + 5.0


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
            if kind == 'ranges':
                self.raw_down = data.get('range.zrange')
            if now - received > self.telemetry_timeout:
                continue
            if kind == 'variance':
                self.variance_samples += 1
                if self.last_variance_at is not None and received - self.last_variance_at > 1.0:
                    self.variance_history.clear()
                self.last_variance_at = received
                values = tuple(float(data['kalman.varP' + axis]) for axis in ('X', 'Y', 'Z'))
                if not all(math.isfinite(v) and v >= 0 for v in values):
                    self.invalid_variance_samples += 1
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
        self.check_startup(now)
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
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[3]), math.sin(self.origin[3])
        # Rotate both translation and yaw into the start-heading frame.
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        local_yaw = math.radians(yaw) - self.origin[3]
        # Remember this so a later /initialpose click has a local pose to
        # anchor its alignment to.
        self.latest_local_pose = (local_x, local_y, local_yaw)
        map_x, map_y, map_yaw = align_planar_pose(
            self.latest_local_pose, self.map_alignment)
        self.latest_map_pose = (map_x, map_y, map_yaw)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = map_x
        msg.pose.pose.position.y = map_y
        msg.pose.pose.position.z = z - self.origin[2]
        msg.pose.pose.orientation.z = math.sin(map_yaw / 2)
        msg.pose.pose.orientation.w = math.cos(map_yaw / 2)
        self.odom_pub.publish(msg)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

    def destroy_node(self):
        self.closing = True
        try:
            if self.flying and not self.dry_run:
                try:
                    self.cf.high_level_commander.land(
                        self.origin[2], self.land_duration)
                    self.get_logger().info('Landing before disconnecting...')
                    time.sleep(self.land_duration)
                except Exception as exc:
                    self.get_logger().warning(f'Landing command failed: {exc}')
            for config in self.logs:
                try:
                    config.stop()
                    config.delete()
                except Exception as exc:
                    self.get_logger().warning(f'Log cleanup failed for {config.name}: {exc}')
        finally:
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
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        raise SystemExit(1) from None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
