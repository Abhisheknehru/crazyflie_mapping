"""ROS 2 localization node using accumulated Multi-ranger scan matching."""
from collections import deque
import math
import sys
import time

import rclpy
from geometry_msgs.msg import (
    Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32

from test_runs.map_loader import load_map, pick_latest_map
from test_runs.scan_matcher import (
    Alignment, CorrelativeScanMatcher, transform_point)


SENSOR_ANGLES = {
    'front': 0.0,
    'left': math.pi / 2.0,
    'back': math.pi,
    'right': -math.pi / 2.0,
}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose_pose(local_pose, alignment):
    """Apply the map<-odometry alignment to an odometry pose."""
    x, y = transform_point(local_pose[:2], alignment)
    yaw = alignment.yaw + local_pose[2]
    return x, y, math.atan2(math.sin(yaw), math.cos(yaw))


class ScanMatchLocalizer(Node):
    def __init__(self):
        super().__init__('scan_match_localizer')
        defaults = (
            ('map_yaml', ''), ('maps_dir', 'maps'), ('update_period', 1.0),
            ('scan_duration', 8.0), ('max_scan_points', 240),
            ('minimum_scan_points', 24), ('maximum_match_error', .18),
            ('xy_search_window', .35), ('yaw_search_window', .26),
            ('xy_search_step', .05), ('yaw_search_step', .052),
            ('alignment_smoothing', .35),
            ('initial_pose_topic', '/crazyflie/scan_match_initialpose'),
            ('auto_initial_pose', True), ('initial_map_x', 0.0),
            ('initial_map_y', 0.0), ('initial_map_yaw_degrees', 0.0),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)

        requested_map = str(self.get_parameter('map_yaml').value).strip()
        if requested_map:
            map_path = requested_map
        else:
            map_path = pick_latest_map(
                str(self.get_parameter('maps_dir').value))
        self.matcher = CorrelativeScanMatcher(load_map(map_path))
        self.alignment = Alignment()
        self.local_pose = None
        self.auto_initial_pose_pending = bool(
            self.get_parameter('auto_initial_pose').value)
        self.scan_points = deque(maxlen=int(
            self.get_parameter('max_scan_points').value))
        self.scan_duration = float(self.get_parameter('scan_duration').value)
        self.minimum_scan_points = int(
            self.get_parameter('minimum_scan_points').value)
        self.maximum_match_error = float(
            self.get_parameter('maximum_match_error').value)
        self.alignment_smoothing = float(
            self.get_parameter('alignment_smoothing').value)
        self.last_result = None

        self.create_subscription(PoseStamped, '/crazyflie/pose',
                                 self.on_pose, 10)
        initial_pose_topic = str(
            self.get_parameter('initial_pose_topic').value)
        self.create_subscription(PoseWithCovarianceStamped, initial_pose_topic,
                                 self.on_initial_pose, 10)
        for face in SENSOR_ANGLES:
            self.create_subscription(
                Range, '/crazyflie/range_' + face,
                self.range_callback(face), qos_profile_sensor_data)
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/crazyflie/scan_matched_pose', 10)
        self.scan_publisher = self.create_publisher(
            PoseArray, '/crazyflie/scan_match_points', 10)
        self.error_publisher = self.create_publisher(
            Float32, '/crazyflie/scan_match_error', 10)
        self.accepted_publisher = self.create_publisher(
            Bool, '/crazyflie/scan_match_accepted', 10)
        self.create_timer(float(self.get_parameter('update_period').value),
                          self.run_localization_cycle)
        self.get_logger().info('Scan matching against ' + map_path)

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if all(math.isfinite(value) for value in values):
            self.local_pose = (float(p.x), float(p.y), yaw_from_quaternion(q))
            if self.auto_initial_pose_pending:
                self.auto_initial_pose_pending = False
                self.set_alignment_from_map_pose((
                    float(self.get_parameter('initial_map_x').value),
                    float(self.get_parameter('initial_map_y').value),
                    math.radians(float(self.get_parameter(
                        'initial_map_yaw_degrees').value))))
                self.get_logger().info(
                    'Automatic initial map pose applied')

    def range_callback(self, face):
        def callback(msg):
            if self.local_pose is None:
                return
            if not (math.isfinite(msg.range) and
                    msg.min_range < msg.range < msg.max_range):
                return
            x, y, yaw = self.local_pose
            beam = yaw + SENSOR_ANGLES[face]
            endpoint = (x + msg.range * math.cos(beam),
                        y + msg.range * math.sin(beam))
            self.scan_points.append((time.monotonic(), endpoint))
        return callback

    def on_initial_pose(self, msg):
        """Convert a user-provided map pose into a map<-odometry alignment."""
        if self.local_pose is None or msg.header.frame_id != 'map':
            self.get_logger().warning(
                'Initial pose ignored: wait for odometry and use the map '
                'frame')
            return
        map_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                    yaw_from_quaternion(msg.pose.pose.orientation))
        self.set_alignment_from_map_pose(map_pose)
        self.auto_initial_pose_pending = False
        self.get_logger().info('Initial scan-matcher alignment accepted')

    def set_alignment_from_map_pose(self, map_pose):
        """Anchor the current odometry pose at a known pose in the map."""
        local_x, local_y, local_yaw = self.local_pose
        alignment_yaw = map_pose[2] - local_yaw
        c, s = math.cos(alignment_yaw), math.sin(alignment_yaw)
        self.alignment = Alignment(
            map_pose[0] - c * local_x + s * local_y,
            map_pose[1] - s * local_x - c * local_y,
            alignment_yaw)
        self.scan_points.clear()

    def run_localization_cycle(self):
        """Run one complete localization cycle in an explicit sequence."""
        # 1. Stop here until odometry is available.
        if self.local_pose is None:
            return

        # 2. Remove old measurements and copy the accumulated sparse scan.
        oldest = time.monotonic() - self.scan_duration
        while self.scan_points and self.scan_points[0][0] < oldest:
            self.scan_points.popleft()
        points = [point for _, point in self.scan_points]
        accepted = False
        mean_error = math.inf

        # 3. Match the scan near the current map-to-odometry alignment.
        if len(points) >= self.minimum_scan_points:
            result = self.matcher.match(
                points, self.alignment,
                xy_window=float(self.get_parameter('xy_search_window').value),
                yaw_window=float(
                    self.get_parameter('yaw_search_window').value),
                xy_step=float(self.get_parameter('xy_search_step').value),
                yaw_step=float(self.get_parameter('yaw_search_step').value))
            self.last_result = result
            mean_error = result.mean_error

            # 4. Accept only a scan that lies sufficiently close to walls.
            if result.mean_error <= self.maximum_match_error:
                dx = result.alignment.x - self.alignment.x
                dy = result.alignment.y - self.alignment.y
                dyaw = math.atan2(
                    math.sin(result.alignment.yaw - self.alignment.yaw),
                    math.cos(result.alignment.yaw - self.alignment.yaw))
                self._smooth_alignment(result.alignment)
                accepted = True
                self.get_logger().info(
                    f'Match accepted: error={result.mean_error:.3f} m, '
                    f'correction=({dx:+.3f}, {dy:+.3f}) m, '
                    f'{math.degrees(dyaw):+.1f} deg')
            else:
                self.get_logger().warning(
                    'Match rejected: mean wall error '
                    f'{result.mean_error:.2f} m')

        # 5. Publish the corrected drone pose and scan for RViz inspection.
        self.publish_localization(points)
        self.error_publisher.publish(Float32(data=float(mean_error)))
        self.accepted_publisher.publish(Bool(data=accepted))

    def _smooth_alignment(self, target):
        a = self.alignment_smoothing
        self.alignment.x += a * (target.x - self.alignment.x)
        self.alignment.y += a * (target.y - self.alignment.y)
        difference = math.atan2(math.sin(target.yaw - self.alignment.yaw),
                                math.cos(target.yaw - self.alignment.yaw))
        self.alignment.yaw += a * difference

    def publish_localization(self, points):
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = compose_pose(self.local_pose, self.alignment)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'map'
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_publisher.publish(pose)

        scan = PoseArray()
        scan.header.stamp, scan.header.frame_id = stamp, 'map'
        for point in points:
            px, py = transform_point(point, self.alignment)
            marker = Pose()
            marker.position.x, marker.position.y = px, py
            marker.orientation.w = 1.0
            scan.poses.append(marker)
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanMatchLocalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (FileNotFoundError, ValueError) as exc:
        print(f'scan_match_localizer failed: {exc}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
