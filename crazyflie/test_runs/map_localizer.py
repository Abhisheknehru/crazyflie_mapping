"""Monte Carlo localization of the drone against a saved 2D map."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from test_runs.map_loader import load_map, pick_latest_map
from test_runs.particle_filter import ParticleFilter

FACE_ANGLES = {'front': 0., 'left': math.pi / 2,
               'back': math.pi, 'right': -math.pi / 2}


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                       1 - 2 * (q.y * q.y + q.z * q.z))


class MapLocalizer(Node):
    def __init__(self):
        super().__init__('map_localizer')
        for name, value in (('maps_dir', 'maps'), ('particle_count', 1200),
                            ('update_period', .5), ('alpha_trans', .05),
                            ('trans_floor', .003), ('alpha_rot', .05),
                            ('rot_floor', .01), ('sigma_base', .05),
                            ('sigma_slope', .05), ('random_inject', .01),
                            ('alignment_spread_threshold', .3),
                            ('alignment_yaw_concentration', .9),
                            ('alignment_hold_ticks', 3),
                            ('alignment_lock_lost_spread', .6),
                            ('resample_warmup_ticks', 6),
                            ('localized_pose_smoothing', .2)):
            self.declare_parameter(name, value)
        particle_count = int(self.get_parameter('particle_count').value)
        self.update_period = float(self.get_parameter('update_period').value)
        self.sigma_base = float(self.get_parameter('sigma_base').value)
        self.sigma_slope = float(self.get_parameter('sigma_slope').value)
        self.random_inject = float(self.get_parameter('random_inject').value)
        self.alignment_spread_threshold = float(
            self.get_parameter('alignment_spread_threshold').value)
        self.alignment_yaw_concentration = float(
            self.get_parameter('alignment_yaw_concentration').value)
        self.alignment_hold_ticks = int(
            self.get_parameter('alignment_hold_ticks').value)
        self.alignment_lock_lost_spread = float(
            self.get_parameter('alignment_lock_lost_spread').value)
        self.resample_warmup_ticks = int(
            self.get_parameter('resample_warmup_ticks').value)
        self.localized_pose_smoothing = float(
            self.get_parameter('localized_pose_smoothing').value)
        if not (particle_count > 0 and
                math.isfinite(self.update_period) and self.update_period > 0 and
                math.isfinite(self.sigma_base) and self.sigma_base > 0 and
                math.isfinite(self.sigma_slope) and self.sigma_slope >= 0 and
                0 <= self.random_inject <= 1 and
                math.isfinite(self.alignment_spread_threshold) and
                self.alignment_spread_threshold > 0 and
                0 < self.alignment_yaw_concentration <= 1 and
                self.alignment_hold_ticks >= 1 and
                math.isfinite(self.alignment_lock_lost_spread) and
                self.alignment_lock_lost_spread > self.alignment_spread_threshold and
                self.resample_warmup_ticks >= 0 and
                0 < self.localized_pose_smoothing <= 1):
            raise ValueError('Invalid particle_count, update_period, sigma, or '
                             'alignment params')

        map_path = pick_latest_map(str(self.get_parameter('maps_dir').value))
        self.localization_map = load_map(map_path)
        self.filter = ParticleFilter(
            self.localization_map, count=particle_count,
            alpha_trans=float(self.get_parameter('alpha_trans').value),
            trans_floor=float(self.get_parameter('trans_floor').value),
            alpha_rot=float(self.get_parameter('alpha_rot').value),
            rot_floor=float(self.get_parameter('rot_floor').value))

        self.last_pose = None
        self.readings = {}
        self.start_time = time.time()
        self.tick_count = 0
        self.min_spread = math.inf

        self.create_subscription(
            PoseStamped, '/crazyflie/pose', self.on_pose, 10)
        for face in FACE_ANGLES:
            self.create_subscription(
                Range, '/crazyflie/range_' + face, self.on_range(face),
                qos_profile_sensor_data)

        self.pose_publisher = self.create_publisher(
            PoseStamped, '/crazyflie/localized_pose', 10)
        self.particles_publisher = self.create_publisher(
            PoseArray, '/crazyflie/particles', 10)
        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.alignment_published = False
        self.converged_streak = 0
        self.smoothed_pose = None
        self.create_timer(self.update_period, self.tick)
        self.get_logger().info(
            f'Localizing against {map_path} with {particle_count} particles')

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return
        yaw = yaw_from_quaternion(q)
        if self.last_pose is not None:
            x0, y0, yaw0 = self.last_pose
            dx, dy = p.x - x0, p.y - y0
            dyaw = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
            # Rotate the raw world-frame delta into the drone's own body
            # frame (its heading at the start of this step) before handing
            # it to the filter. This makes the motion model correct
            # regardless of any unknown offset between this session's
            # arbitrary "zero heading" and the saved map's own frame --
            # each particle then re-rotates this body-frame motion by its
            # OWN yaw hypothesis in predict().
            c, s = math.cos(yaw0), math.sin(yaw0)
            forward = c * dx + s * dy
            lateral = -s * dx + c * dy
            self.filter.predict(forward, lateral, dyaw)
        self.last_pose = (p.x, p.y, yaw)

    def on_range(self, face):
        def callback(msg):
            if (math.isfinite(msg.range) and math.isfinite(msg.max_range) and
                    0 < msg.range <= msg.max_range <= 4.):
                self.readings[face] = float(msg.range)
        return callback

    def tick(self):
        self.tick_count += 1
        if self.readings:
            self.filter.update(dict(self.readings), FACE_ANGLES)
            # Skip resampling for the first few ticks: resampling throws
            # away particle diversity in favor of whatever looks best so
            # far, and a single round of readings from a symmetric-looking
            # starting spot isn't enough evidence to safely commit to one
            # heading over its mirror image. Weight evidence keeps
            # accumulating multiplicatively during warmup regardless
            # (update() still runs every tick); only the collapse is
            # deferred.
            if self.tick_count > self.resample_warmup_ticks:
                self.filter.resample(self.random_inject)
        current_spread = self.spread()
        self.min_spread = min(self.min_spread, current_spread)
        if self.alignment_published:
            # A published alignment can still be wrong -- e.g. a symmetric
            # corridor where all particles confidently agree on a heading
            # that's 180 degrees off from reality. Agreement among
            # particles isn't the same as correctness, so if the filter's
            # own confidence later collapses (spread grows a lot), treat
            # the alignment as untrustworthy again and allow it to
            # reconverge and republish -- e.g. once the drone passes
            # distinguishing geometry that breaks the symmetry.
            if current_spread > self.alignment_lock_lost_spread:
                self.alignment_published = False
                self.converged_streak = 0
                self.get_logger().warning(
                    f'Localization lock lost (spread={current_spread:.2f}m); '
                    'radio_bridge keeps its last alignment until this '
                    'reconverges and republishes')
        else:
            yaw_concentration = self.filter.yaw_concentration()
            if (current_spread < self.alignment_spread_threshold and
                    yaw_concentration > self.alignment_yaw_concentration):
                self.converged_streak += 1
            else:
                self.converged_streak = 0
            if self.converged_streak >= self.alignment_hold_ticks:
                self.publish_alignment(yaw_concentration)
        self.publish()

    def publish_alignment(self, yaw_concentration):
        x, y, yaw = self.filter.estimate()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.initialpose_publisher.publish(msg)
        self.alignment_published = True
        self.get_logger().info(
            f'Localization converged (spread={self.min_spread:.2f}m, '
            f'yaw_concentration={yaw_concentration:.2f}, held for '
            f'{self.alignment_hold_ticks} ticks); published alignment at '
            f'({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg) to /initialpose')

    def spread(self):
        x, y, _ = self.filter.estimate()
        total = sum(p.weight for p in self.filter.particles) or 1.0
        var_x = sum(p.weight * (p.x - x) ** 2
                    for p in self.filter.particles) / total
        var_y = sum(p.weight * (p.y - y) ** 2
                    for p in self.filter.particles) / total
        return math.sqrt(var_x + var_y)

    def smoothed_estimate(self):
        # The raw weighted mean is recomputed from scratch every tick, and
        # every tick resamples (drawing a fresh random particle set) -- so
        # even a genuinely stable belief produces a noisy raw average
        # tick-to-tick, most visibly in yaw. Blend each new estimate into
        # the previously PUBLISHED value instead of republishing the raw
        # number directly, so real changes still come through but
        # resampling noise gets damped out. This only affects what's
        # published/displayed -- the alignment gate and internal filter
        # state still use the raw self.filter.estimate() directly.
        x, y, yaw = self.filter.estimate()
        if self.smoothed_pose is None:
            self.smoothed_pose = (x, y, yaw)
            return self.smoothed_pose
        sx, sy, syaw = self.smoothed_pose
        a = self.localized_pose_smoothing
        sx += a * (x - sx)
        sy += a * (y - sy)
        sin_s = math.sin(syaw) + a * (math.sin(yaw) - math.sin(syaw))
        cos_s = math.cos(syaw) + a * (math.cos(yaw) - math.cos(syaw))
        syaw = math.atan2(sin_s, cos_s)
        self.smoothed_pose = (sx, sy, syaw)
        return self.smoothed_pose

    def publish(self):
        x, y, yaw = self.smoothed_estimate()
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.z = math.sin(yaw / 2)
        pose_msg.pose.orientation.w = math.cos(yaw / 2)
        self.pose_publisher.publish(pose_msg)

        array_msg = PoseArray()
        array_msg.header.stamp = stamp
        array_msg.header.frame_id = 'map'
        for particle in self.filter.particles:
            pose = Pose()
            pose.position.x = particle.x
            pose.position.y = particle.y
            pose.orientation.z = math.sin(particle.yaw / 2)
            pose.orientation.w = math.cos(particle.yaw / 2)
            array_msg.poses.append(pose)
        self.particles_publisher.publish(array_msg)

    def summarize(self):
        x, y, yaw = self.filter.estimate()
        spread = self.spread()
        duration = time.time() - self.start_time
        faces = ', '.join(sorted(self.readings)) or 'none'
        pose_status = 'received' if self.last_pose is not None else 'never received'
        print(
            'map_localizer summary:\n'
            f'  ran {duration:.1f}s, {self.tick_count} update ticks\n'
            f'  final estimate: x={x:.2f}m  y={y:.2f}m  yaw={math.degrees(yaw):.0f} deg\n'
            f'  final spread: {spread:.2f}m  (best seen: {self.min_spread:.2f}m; '
            f'lower = more confident)\n'
            f'  yaw concentration: {self.filter.yaw_concentration():.2f} '
            f'(1.0 = fully agreed, near 0 = split/ambiguous heading)\n'
            f'  range faces last seen: {faces}\n'
            f'  odometry (/crazyflie/pose): {pose_status}\n'
            f'  alignment published to radio_bridge: {self.alignment_published}',
            flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapLocalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (FileNotFoundError, ValueError) as exc:
        print(f'map_localizer failed to start: {exc}', file=sys.stderr, flush=True)
    finally:
        if node is not None:
            node.summarize()
            node.destroy_node()
        rclpy.try_shutdown()
