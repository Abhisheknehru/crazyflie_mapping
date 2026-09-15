import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import socket
import json
import math
import struct


COMBINED_FORMAT = '<IHHHHffff'
COMBINED_SIZE = struct.calcsize(COMBINED_FORMAT)
LEGACY_FORMAT = '<IHHHHf'
LEGACY_SIZE = struct.calcsize(LEGACY_FORMAT)


def to_metres(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        return None
    if value > 10.0:
        value /= 1000.0
    return value


def parse_range_packet(data):
    """Return front, left, back, right in metres, or None for bad packets."""
    front = right = back = left = None

    try:
        payload = data.decode('utf-8').strip()
    except UnicodeDecodeError:
        payload = None

    if payload and (payload.startswith('{') or ',' in payload):
        try:
            if payload.startswith('{'):
                ranges = json.loads(payload)
                front = ranges.get('front')
                left = ranges.get('left')
                back = ranges.get('back')
                right = ranges.get('right')
            else:
                fields = [float(value) for value in payload.split(',')]
                if len(fields) < 4:
                    return None
                front, back, left, right = fields[:4]
        except (ValueError, json.JSONDecodeError):
            return None
    elif len(data) >= COMBINED_SIZE:
        _, front, right, back, left, *_pose = struct.unpack(
            COMBINED_FORMAT, data[:COMBINED_SIZE]
        )
    elif len(data) >= LEGACY_SIZE:
        _, front, right, back, left, _hz = struct.unpack(
            LEGACY_FORMAT, data[:LEGACY_SIZE]
        )
    else:
        return None

    readings = tuple(to_metres(value) for value in (front, left, back, right))
    if any(value is None for value in readings):
        return None
    return readings

class SeeedC3UDPBridge(Node):
    def __init__(self):
        super().__init__('seeed_c3_udp_bridge')

        # Setup publishers for each individual face matching the mapper node
        self.pub_front = self.create_publisher(Range, '/crazyflie/range_front', 10)
        self.pub_left = self.create_publisher(Range, '/crazyflie/range_left', 10)
        self.pub_back = self.create_publisher(Range, '/crazyflie/range_back', 10)
        self.pub_right = self.create_publisher(Range, '/crazyflie/range_right', 10)

        # Network configurations
        self.udp_ip = "0.0.0.0" # Listen on all local interfaces
        self.udp_port = 5005

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False) # Set to non-blocking to prevent freezing ROS processing loops

        self.get_logger().info(f"UDP Bridge active. Listening for Seeed C3 packets on port {self.udp_port}...")

        # Poll the socket interface at 100Hz 
        self.create_timer(0.01, self.receive_udp_data)

    def receive_udp_data(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            ranges = parse_range_packet(data)
            if ranges is None:
                self.get_logger().error(f"Malformed payload dropped from {addr}: {len(data)} bytes")
                return

            front, left, back, right = ranges
            self.publish_range_msg(self.pub_front, front)
            self.publish_range_msg(self.pub_left, left)
            self.publish_range_msg(self.pub_back, back)
            self.publish_range_msg(self.pub_right, right)
            self.get_logger().info(
                f"Received range data: front={front:.3f}, left={left:.3f}, "
                f"back={back:.3f}, right={right:.3f}"
            )

        except BlockingIOError:
            # Safe catch: Raised when no incoming packets are queued up in the buffer yet
            pass
        except (ValueError, KeyError, struct.error) as err:
            self.get_logger().error(f"Malformed payload dropped: {err}")

    def publish_range_msg(self, publisher, distance_value):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Multi-Ranger configurations (VL53L1X Time-of-Flight Sensors)
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.47 # ~27 degrees field-of-view cone in radians
        msg.min_range = 0.01     # 1cm minimum range
        msg.max_range = 4.0      # 4 meters maximum tracking range
        msg.range = float(distance_value)

        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SeeedC3UDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
