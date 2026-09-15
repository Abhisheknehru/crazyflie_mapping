"""Initial range mapper: supplied odometry, no SLAM or loop closure."""
import bisect
from collections import deque
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from test_runs.grid_map import GridMap


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MultiRangerMapper(Node):
    def __init__(self):
        super().__init__('multiranger_mapper')
        for name, value in (('resolution', .020), ('size', 10.0),
                            ('pose_topic', '/of/pose'),
                            ('max_pose_age', .15), ('max_pose_gap', .3),
                            ('sensor_offset', .03)):
            self.declare_parameter(name, value)
        self.grid = GridMap(float(self.get_parameter('resolution').value),
                            float(self.get_parameter('size').value))
        self.age = float(self.get_parameter('max_pose_age').value)
        self.gap = float(self.get_parameter('max_pose_gap').value)
        self.offset = float(self.get_parameter('sensor_offset').value)
        if not (math.isfinite(self.age) and 0 < self.age <= 1 and
                math.isfinite(self.gap) and 0 < self.gap <= 1 and
                math.isfinite(self.offset) and 0 <= self.offset <= .2):
            raise ValueError('Invalid pose age, pose gap or sensor offset')
        self.poses = deque(maxlen=100)
        self.pose_times = deque(maxlen=100)
        self.pending = deque(maxlen=100)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value,
                                 self.on_pose, 10)
        for face, angle in (('front', 0.), ('left', math.pi / 2),
                            ('back', math.pi), ('right', -math.pi / 2)):
            self.create_subscription(Range, '/crazyflie/range_' + face,
                                     self.on_range(angle), qos_profile_sensor_data)
        self.create_timer(.05, self.process)
        self.create_timer(.5, self.publish)
        self.get_logger().info('Mapping to /map; fixed 10 m default grid, no loop closure.')

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in values):
            return
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if abs(norm - 1) > .01:
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        stamp = seconds(msg.header.stamp)
        if self.pose_times and stamp <= self.pose_times[-1]:
            return
        self.poses.append((stamp, p.x, p.y, yaw))
        self.pose_times.append(stamp)

    def interpolate(self, stamp):
        """Pose at an exact instant, or None when it cannot be bracketed."""
        index = bisect.bisect_left(self.pose_times, stamp)
        if index == 0:
            _, x, y, yaw = self.poses[0]
            return (x, y, yaw) if stamp == self.pose_times[0] else None
        if index >= len(self.poses):
            return None
        t0, x0, y0, yaw0 = self.poses[index - 1]
        t1, x1, y1, yaw1 = self.poses[index]
        span = t1 - t0
        if span > self.gap:
            return None
        fraction = (stamp - t0) / span
        # Shortest arc, so interpolation across +/-pi does not sweep backwards.
        turn = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        return (x0 + fraction * (x1 - x0), y0 + fraction * (y1 - y0),
                yaw0 + fraction * turn)

    def on_range(self, angle):
        def callback(msg):
            # Both real near-minimum readings and clamped faults are skipped.
            if (math.isfinite(msg.range) and math.isfinite(msg.max_range) and
                    math.isfinite(msg.min_range) and
                    0 < msg.min_range and msg.min_range + 1e-5 < msg.range <= msg.max_range <= 4.0):
                self.pending.append((seconds(msg.header.stamp), angle,
                                     float(msg.range), msg.range < msg.max_range - 1e-5))
        return callback

    def process(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for _ in range(len(self.pending)):
            stamp, angle, distance, hit = self.pending.popleft()
            if stamp <= 0 or now - stamp > 1 or stamp > now + .1:
                continue
            if not self.poses or self.pose_times[-1] < stamp:
                # Hold the reading for the pose that has not arrived yet, unless
                # the pose stream has stalled far enough that it never will.
                if not self.poses or stamp - self.pose_times[-1] <= self.age:
                    self.pending.append((stamp, angle, distance, hit))
                continue
            pose = self.interpolate(stamp)
            if pose is None:
                continue
            x, y, yaw = pose
            direction = yaw + angle
            self.grid.ray(x + self.offset * math.cos(direction),
                          y + self.offset * math.sin(direction), direction, distance, hit)

    def publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.grid.resolution
        msg.info.width = msg.info.height = self.grid.width
        msg.info.origin.position.x = msg.info.origin.position.y = self.grid.origin
        msg.info.origin.orientation.w = 1.
        msg.data = self.grid.occupancy()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRangerMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
