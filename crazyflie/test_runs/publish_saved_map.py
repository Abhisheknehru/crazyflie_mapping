"""Publish an already-saved map to /map once, for viewing in RViz.

No live sensors, no mapping -- just loads the newest maze_*.yaml and
republishes it as a latched OccupancyGrid so it doesn't need
multiranger_mapper running to be visible.
"""
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from test_runs.map_loader import load_map, pick_latest_map


class SavedMapPublisher(Node):
    def __init__(self):
        super().__init__('publish_saved_map')
        self.declare_parameter('maps_dir', 'maps')
        map_path = pick_latest_map(str(self.get_parameter('maps_dir').value))
        localization_map = load_map(map_path)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = self.create_publisher(OccupancyGrid, '/map', map_qos)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = localization_map.resolution
        msg.info.width = msg.info.height = localization_map.width
        msg.info.origin.position.x = msg.info.origin.position.y = localization_map.origin
        msg.info.origin.orientation.w = 1.
        msg.data = localization_map.occupancy
        publisher.publish(msg)
        self.get_logger().info(
            f'Published {map_path} to /map (latched) -- add a Map display '
            'in RViz, this node just needs to stay running')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SavedMapPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except FileNotFoundError as exc:
        print(f'publish_saved_map failed: {exc}', file=sys.stderr, flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
