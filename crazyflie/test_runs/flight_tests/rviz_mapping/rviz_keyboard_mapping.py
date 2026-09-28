#!/usr/bin/env python3
"""Fly with the existing keyboard UI and publish its point cloud to RViz."""
from collections import deque
import math
import signal
import sys

import numpy as np
import rclpy
from cflib.utils import uri_helper
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from test_runs.point_cloud_test import MainWindow, QT6, QtCore, QtWidgets


MAX_POINTS = 50000
POSITION_FREEZE_WINDOW = 20
POSITION_FREEZE_EPSILON = 1e-6


class MappingPublisher(Node):
    def __init__(self):
        super().__init__('rviz_keyboard_mapping')
        self.declare_parameter(
            'uri', uri_helper.uri_from_env(
                default='radio://0/80/2M/E7E7E7E7E7'))
        self.cloud_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/crazyflie/point_cloud', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/crazyflie/pose', 10)
        self.range_pubs = {
            face: self.create_publisher(
                Range, '/crazyflie/range_' + face, 10)
            for face in ('front', 'left', 'back', 'right')
        }

    def publish_data(self, position, yaw_degrees, points, ranges):
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        yaw = math.radians(float(yaw_degrees))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

        if ranges is not None:
            for face, publisher in self.range_pubs.items():
                raw = float(ranges[face])
                msg = Range()
                msg.header.stamp = stamp
                msg.header.frame_id = 'range_' + face
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = math.radians(27.0)
                msg.min_range = 0.02
                msg.max_range = 4.0
                if not math.isfinite(raw) or raw <= 20 or raw >= 32767:
                    msg.range = msg.max_range
                else:
                    msg.range = min(msg.max_range, raw / 1000.0)
                publisher.publish(msg)

        if len(points):
            header = Header(stamp=stamp, frame_id='map')
            cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
            self.cloud_pub.publish(cloud)


class RvizMappingWindow(MainWindow):
    def __init__(self, uri, publisher):
        self.publisher = publisher
        self.cloud_points = np.empty((0, 3), dtype=np.float32)
        self.latest_yaw = 0.0
        self.latest_ranges = None
        self.ros_cleaned_up = False
        self.position_history = deque(maxlen=POSITION_FREEZE_WINDOW)
        self.position_frozen = False
        super().__init__(uri)
        self.setWindowTitle('Crazyflie keyboard control + RViz mapping')

        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.publish_ros)
        self.ros_timer.setInterval(100)
        self.ros_timer.start()

    def meas_data(self, timestamp, data, logconf):
        super().meas_data(timestamp, data, logconf)
        self.latest_yaw = float(data['stabilizer.yaw'])
        measurement = {
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
            'front': data['range.front'],
            'back': data['range.back'],
            'up': data['range.up'],
            'down': data['range.zrange'],
            'left': data['range.left'],
            'right': data['range.right'],
        }
        self.latest_ranges = {
            face: measurement[face]
            for face in ('front', 'left', 'back', 'right')
        }
        new_points = self.canvas.rotate_and_create_points(measurement)
        if new_points:
            batch = np.asarray(new_points, dtype=np.float32).reshape(-1, 3)
            self.cloud_points = np.concatenate((self.cloud_points, batch))
            if len(self.cloud_points) > MAX_POINTS:
                self.cloud_points = self.cloud_points[-MAX_POINTS:]

    def pos_data(self, timestamp, data, logconf):
        super().pos_data(timestamp, data, logconf)
        # stateEstimate (optical flow / Kalman position) can stall -- log
        # config still delivers messages on schedule, but the underlying
        # value stops updating (dead/loose Flow deck, stalled estimator).
        # Real telemetry always has some sub-mm jitter even hovering still;
        # bit-for-bit identical readings over a whole window means the
        # sensor isn't actually reporting anything new. That's exactly the
        # "flies on stale data" hazard, so land immediately rather than
        # keep trusting it.
        sample = (data['stateEstimate.x'], data['stateEstimate.y'],
                  data['stateEstimate.z'])
        self.position_history.append(sample)
        if not self.position_frozen and self._position_is_frozen():
            self.position_frozen = True
            print('SAFETY LAND: stateEstimate stopped changing while '
                  'telemetry kept arriving -- landing now.')
            self.startLanding(close_after=False)

    def _position_is_frozen(self):
        if len(self.position_history) < self.position_history.maxlen:
            return False
        columns = zip(*self.position_history)
        return all(max(values) - min(values) < POSITION_FREEZE_EPSILON
                   for values in columns)

    def publish_ros(self):
        rclpy.spin_once(self.publisher, timeout_sec=0.0)
        self.publisher.publish_data(
            self.canvas.last_pos, self.latest_yaw, self.cloud_points,
            self.latest_ranges)

    def closeEvent(self, event):
        super().closeEvent(event)
        if event.isAccepted() and not self.ros_cleaned_up:
            self.ros_cleaned_up = True
            self.ros_timer.stop()
            
            self.publisher.destroy_node()
            rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    publisher = MappingPublisher()
    uri = str(publisher.get_parameter('uri').value)
    app = QtWidgets.QApplication([sys.argv[0]])
    window = RvizMappingWindow(uri, publisher)

    def handle_sigint(signum, frame):
        print('\nCtrl+C received; starting controlled landing...')
        window.startLanding(close_after=True)

    signal.signal(signal.SIGINT, handle_sigint)
    window.show()
    if QT6:
        app.exec()
    else:
        app.exec_()


if __name__ == '__main__':
    main()
