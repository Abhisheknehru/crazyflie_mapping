"""Replay a recorded rosbag2 through MultiRangerMapper, offline.

Feeds /crazyflie/pose and /crazyflie/range_* messages from a bag straight
into the node's callbacks (no rclpy.spin, no live topics) and periodically
reports occupancy near a chosen point -- e.g. to check that a wall's
occupancy value stays high instead of decaying as the drone moves away
from it (see multiranger_mapper.py's trust_near/trust_far hysteresis).
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Range

from test_runs.multiranger_mapper import MultiRangerMapper

FACES = ('front', 'left', 'back', 'right')


def read_bag(path):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='sqlite3'),
                ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def replay(bag_path, report_every, watch_xy):
    rclpy.init(args=[])
    node = MultiRangerMapper()
    face_callback = {
        'front': node.on_range('front', 0.),
        'left': node.on_range('left', math.pi / 2),
        'back': node.on_range('back', math.pi),
        'right': node.on_range('right', -math.pi / 2),
    }
    messages = 0
    for topic, msg in read_bag(bag_path):
        if topic == '/crazyflie/pose' and isinstance(msg, PoseStamped):
            node.on_pose(msg)
        elif isinstance(msg, Range):
            for face in FACES:
                if topic == '/crazyflie/range_' + face:
                    face_callback[face](msg)
                    break
        node.process()
        messages += 1
        if messages % report_every == 0:
            cell = node.grid.cell(*watch_xy)
            value = node.grid.occupancy()[cell] if cell is not None else -1
            print(f'msg {messages}: occupancy at {watch_xy} = {value}, '
                  f'face_trusted = {node.face_trusted}')
    cell = node.grid.cell(*watch_xy)
    value = node.grid.occupancy()[cell] if cell is not None else -1
    print(f'final: occupancy at {watch_xy} = {value}')
    node.destroy_node()
    rclpy.try_shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_path')
    parser.add_argument('--report-every', type=int, default=50)
    parser.add_argument('--watch-x', type=float, required=True,
                         help='x of the cell to track occupancy for, e.g. a known wall')
    parser.add_argument('--watch-y', type=float, required=True)
    parsed = parser.parse_args(args)
    replay(parsed.bag_path, parsed.report_every, (parsed.watch_x, parsed.watch_y))


if __name__ == '__main__':
    sys.exit(main())
