"""Simple occupancy mapper using onboard stateEstimate and Multi-ranger data."""
import bisect
from collections import deque
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Range

from test_runs.grid_map import GridMap
from test_runs.map_export import save_grid


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MultiRangerMapper(Node):
    def __init__(self):
        super().__init__('multiranger_mapper')
        for name, value in (('resolution', .05), ('size', 2.0),
                            ('pose_topic', '/crazyflie/pose'),
                            ('max_pose_gap', .3), ('sensor_offset', .03),
                            ('trust_near', .3), ('trust_far', .4),
                            ('map_save_prefix', 'maps/maze')):
            self.declare_parameter(name, value)
        self.grid = GridMap(float(self.get_parameter('resolution').value),
                            float(self.get_parameter('size').value))
        self.max_pose_gap = float(self.get_parameter('max_pose_gap').value)
        self.sensor_offset = float(self.get_parameter('sensor_offset').value)
        self.trust_near = float(self.get_parameter('trust_near').value)
        self.trust_far = float(self.get_parameter('trust_far').value)
        if not (math.isfinite(self.max_pose_gap) and
                0 < self.max_pose_gap <= 1 and
                math.isfinite(self.sensor_offset) and
                0 <= self.sensor_offset <= .2 and
                math.isfinite(self.trust_near) and
                math.isfinite(self.trust_far) and
                0 < self.trust_near < self.trust_far):
            raise ValueError(
                'Invalid pose gap, sensor offset, or trust thresholds')
        self.poses = deque(maxlen=100)
        self.pose_times = deque(maxlen=100)
        self.pending = deque(maxlen=100)
        self.face_trusted = {face: False
                              for face in ('front', 'left', 'back', 'right')}
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', map_qos)
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self.on_pose, 10)
        for face, angle in (('front', 0.), ('left', math.pi / 2),
                            ('back', math.pi), ('right', -math.pi / 2)):
            self.create_subscription(
                Range, '/crazyflie/range_' + face,
                self.on_range(face, angle), qos_profile_sensor_data)
        self.create_timer(.05, self.process)
        self.create_timer(.5, self.publish)
        self.get_logger().info(
            'Simple mapping active: /crazyflie/pose + Multi-ranger -> /map')

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        stamp = seconds(msg.header.stamp)
        if (msg.header.frame_id != 'map' or stamp <= 0 or
                not all(math.isfinite(value) for value in values)):
            return
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if abs(norm - 1.) > .01:
            return
        if self.pose_times and stamp <= self.pose_times[-1]:
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1-2*(q.y*q.y + q.z*q.z))
        self.poses.append((stamp, p.x, p.y, yaw))
        self.pose_times.append(stamp)

    def on_range(self, face, angle):
        def callback(msg):
            stamp = seconds(msg.header.stamp)
            if (stamp > 0 and math.isfinite(msg.range) and
                    math.isfinite(msg.min_range) and
                    math.isfinite(msg.max_range) and
                    0 < msg.min_range < msg.range <= msg.max_range <= 4.):
                hit = msg.range < msg.max_range - 1e-5
                self.pending.append(
                    (stamp, angle, face, float(msg.range), hit))
        return callback

    def update_trust(self, face, distance):
        trusted = self.face_trusted[face]
        if trusted and distance > self.trust_far:
            trusted = False
        elif not trusted and distance < self.trust_near:
            trusted = True
        self.face_trusted[face] = trusted
        return trusted

    def interpolate_pose(self, stamp):
        if not self.poses:
            return None
        index = bisect.bisect_left(self.pose_times, stamp)
        if index == 0:
            return self.poses[0][1:] if stamp == self.pose_times[0] else None
        if index >= len(self.poses):
            return None
        t0, x0, y0, yaw0 = self.poses[index - 1]
        t1, x1, y1, yaw1 = self.poses[index]
        if t1 - t0 > self.max_pose_gap:
            return None
        fraction = (stamp - t0) / (t1 - t0)
        turn = math.atan2(math.sin(yaw1-yaw0), math.cos(yaw1-yaw0))
        return (x0 + fraction*(x1-x0), y0 + fraction*(y1-y0),
                yaw0 + fraction*turn)

    def process(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for _ in range(len(self.pending)):
            stamp, angle, face, distance, hit = self.pending.popleft()
            if now - stamp > 1. or stamp > now + .1:
                continue
            if not self.pose_times or self.pose_times[-1] < stamp:
                self.pending.append((stamp, angle, face, distance, hit))
                continue
            pose = self.interpolate_pose(stamp)
            if pose is None:
                continue
            x, y, yaw = pose
            direction = yaw + angle
            # ToF hits get noisier the farther they are, so each face only
            # trusts a hit once it's near enough (hysteresis, per face, so
            # a face right at the boundary doesn't flicker). An untrusted
            # reading still clears free space, but only up to trust_near --
            # never far enough to erode a wall it can't actually resolve.
            trusted = self.update_trust(face, distance)
            trusted_hit = hit and trusted
            ray_distance = distance if trusted_hit else min(
                distance, self.trust_near)
            self.grid.ray(
                x + self.sensor_offset * math.cos(direction),
                y + self.sensor_offset * math.sin(direction),
                direction, ray_distance, trusted_hit)

    def publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.grid.resolution
        msg.info.width = msg.info.height = self.grid.width
        msg.info.origin.position.x = msg.info.origin.position.y = self.grid.origin
        msg.info.origin.orientation.w = 1.
        msg.data = self.grid.occupancy()
        self.map_publisher.publish(msg)

    def save_on_shutdown(self):
        if not self.grid.seen:
            print('Map save skipped: no map observations were collected.',
                  flush=True)
            return
        try:
            pgm, yaml = save_grid(
                self.grid,
                str(self.get_parameter('map_save_prefix').value))
        except (OSError, ValueError) as exc:
            print(f'Map save failed: {exc}', file=sys.stderr, flush=True)
            return
        print(f'Map saved: {pgm}\nMap metadata: {yaml}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRangerMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.save_on_shutdown()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
