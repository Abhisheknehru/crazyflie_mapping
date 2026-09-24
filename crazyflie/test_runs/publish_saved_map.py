"""Publish one saved occupancy map for RViz without running a mapper."""
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
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('maps_dir', 'maps')
        requested = str(self.get_parameter('map_yaml').value).strip()
        map_path = requested or pick_latest_map(
            str(self.get_parameter('maps_dir').value))
        saved_map = load_map(map_path)

        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.resolution = saved_map.resolution
        message.info.width = message.info.height = saved_map.width
        message.info.origin.position.x = saved_map.origin
        message.info.origin.position.y = saved_map.origin
        message.info.origin.orientation.w = 1.0
        message.data = saved_map.occupancy
        self.publisher.publish(message)
        self.get_logger().info(f'Published {map_path} on /map')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SavedMapPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (FileNotFoundError, ValueError) as exc:
        print(f'publish_saved_map failed: {exc}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
