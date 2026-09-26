"""Guarded A* waypoint follower using the scan-matched map pose."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32

from test_runs.a_star import AStarPlanner, body_velocity
from test_runs.map_loader import load_map, pick_latest_map


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        defaults = (
            ('map_yaml', ''), ('maps_dir', 'maps'),
            ('clearance', .10), ('required_accepted_matches', 5),
            ('orthogonal_paths', True),
            ('maximum_localization_error', .10),
            ('localization_timeout', 2.0),
            ('goal_topic', '/crazyflie/navigation_goal'),
            ('maximum_speed', .10), ('control_gain', .8),
            ('waypoint_tolerance', .08),
            ('obstacle_stop_distance', .15),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)

        requested = str(self.get_parameter('map_yaml').value).strip()
        map_path = requested or pick_latest_map(
            str(self.get_parameter('maps_dir').value))
        self.planner = AStarPlanner(
            load_map(map_path),
            clearance=float(self.get_parameter('clearance').value),
            orthogonal_paths=bool(
                self.get_parameter('orthogonal_paths').value))
        self.required_matches = int(
            self.get_parameter('required_accepted_matches').value)
        self.maximum_error = float(
            self.get_parameter('maximum_localization_error').value)
        self.localization_timeout = float(
            self.get_parameter('localization_timeout').value)
        self.maximum_speed = float(
            self.get_parameter('maximum_speed').value)
        self.control_gain = float(self.get_parameter('control_gain').value)
        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value)
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)

        self.pose = None
        self.pose_received_at = None
        self.match_error = math.inf
        self.accepted_streak = 0
        self.enabled = False
        self.waypoints = []
        self.waypoint_index = 0
        self.ranges = {}

        self.create_subscription(
            PoseStamped, '/crazyflie/scan_matched_pose', self.on_pose, 10)
        self.create_subscription(
            Bool, '/crazyflie/scan_match_accepted', self.on_accepted, 10)
        self.create_subscription(
            Float32, '/crazyflie/scan_match_error', self.on_error, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value),
            self.on_goal, 10)
        self.create_subscription(
            Bool, '/crazyflie/navigation_enable', self.on_enable, 10)
        for face in ('front', 'left', 'back', 'right'):
            self.create_subscription(
                Range, '/crazyflie/range_' + face,
                self.range_callback(face), 10)

        self.path_publisher = self.create_publisher(
            Path, '/crazyflie/planned_path', 10)
        map_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.inflated_map_publisher = self.create_publisher(
            OccupancyGrid, '/crazyflie/inflated_map', map_qos)
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/crazyflie/navigation_velocity', 10)
        self.publish_inflated_map()
        self.create_timer(.1, self.control_cycle)
        self.get_logger().info(
            f'A* ready with {self.planner.clearance:.2f} m clearance; '
            'select a goal, then explicitly enable navigation')

    def publish_inflated_map(self):
        """Publish the exact blocked/free grid used by A*."""
        source = self.planner.map
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.resolution = source.resolution
        message.info.width = message.info.height = source.width
        message.info.origin.position.x = source.origin
        message.info.origin.position.y = source.origin
        message.info.origin.orientation.w = 1.0
        message.data = [100 if blocked else 0
                        for blocked in self.planner.blocked]
        self.inflated_map_publisher.publish(message)

    def on_pose(self, message):
        if message.header.frame_id != 'map':
            return
        x = float(message.pose.position.x)
        y = float(message.pose.position.y)
        q = message.pose.orientation
        values = (x, y, q.x, q.y, q.z, q.w)
        if all(math.isfinite(value) for value in values):
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = (x, y, yaw)
            self.pose_received_at = time.monotonic()

    def on_error(self, message):
        self.match_error = float(message.data)

    def on_accepted(self, message):
        if message.data and self.match_error <= self.maximum_error:
            self.accepted_streak += 1
        else:
            self.accepted_streak = 0

    def range_callback(self, face):
        def callback(message):
            self.ranges[face] = float(message.range)
        return callback

    def localization_is_ready(self):
        return (
            self.pose is not None and
            self.pose_received_at is not None and
            time.monotonic() - self.pose_received_at <=
            self.localization_timeout and
            self.accepted_streak >= self.required_matches and
            self.match_error <= self.maximum_error
        )

    def on_goal(self, message):
        """Validate localization, plan, publish, and remain disabled."""
        self.stop_navigation('New goal received; navigation stopped')
        if message.header.frame_id != 'map':
            self.get_logger().error('Goal rejected: frame must be map')
            return
        if not self.localization_is_ready():
            self.get_logger().error(
                'Goal rejected: localization is not stable')
            return
        goal = (float(message.pose.position.x),
                float(message.pose.position.y))
        try:
            waypoints = self.planner.plan(self.pose[:2], goal)
        except ValueError as error:
            self.get_logger().error(f'Goal rejected: {error}')
            return
        self.publish_path(waypoints, message)
        self.waypoints = waypoints
        self.waypoint_index = 1 if len(waypoints) > 1 else 0
        self.get_logger().info(
            f'Planned {len(waypoints)} simplified waypoints to '
            f'({goal[0]:.2f}, {goal[1]:.2f}); send navigation_enable=true '
            'to execute')

    def on_enable(self, message):
        if not message.data:
            self.stop_navigation('Navigation cancelled')
            return
        if not self.waypoints:
            self.get_logger().error('Navigation not enabled: no planned path')
            return
        if not self.localization_is_ready():
            self.get_logger().error(
                'Navigation not enabled: localization is not stable')
            return
        self.enabled = True
        self.get_logger().warning(
            f'NAVIGATION ENABLED at maximum {self.maximum_speed:.2f} m/s')

    def control_cycle(self):
        """Follow the path with body-frame horizontal velocity commands."""
        if not self.enabled:
            return
        if not self.localization_is_ready():
            self.stop_navigation(
                'Localization lost; horizontal motion stopped')
            return
        if self.waypoint_index >= len(self.waypoints):
            self.stop_navigation('Goal reached')
            return

        target = self.waypoints[self.waypoint_index]
        forward, left, distance = body_velocity(
            self.pose, target, self.control_gain, self.maximum_speed)
        if distance <= self.waypoint_tolerance:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.stop_navigation('Goal reached')
            return
        if self.obstacle_blocks(forward, left):
            self.stop_navigation(
                'Unexpected nearby obstacle; horizontal motion stopped')
            return

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = forward
        command.twist.linear.y = left
        self.velocity_publisher.publish(command)

    def obstacle_blocks(self, forward, left):
        limit = self.obstacle_stop_distance
        checks = ((forward > 0, 'front'), (forward < 0, 'back'),
                  (left > 0, 'left'), (left < 0, 'right'))
        return any(direction and self.ranges.get(face, math.inf) < limit
                   for direction, face in checks)

    def publish_stop(self):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        self.velocity_publisher.publish(command)

    def stop_navigation(self, reason):
        was_enabled = self.enabled
        self.enabled = False
        self.publish_stop()
        if was_enabled:
            self.get_logger().warning(reason)

    def publish_path(self, waypoints, goal_message):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for index, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(waypoints):
                next_x, next_y = waypoints[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation = goal_message.pose.orientation
            path.poses.append(pose)
        self.path_publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AStarPlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (FileNotFoundError, ValueError) as exc:
        print(f'a_star_planner failed: {exc}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
