#!/usr/bin/env python3
"""Record the four Multi-Ranger streams to a CSV for event characterisation.

Subscribes to the range topics published by udp.py and writes one row per
sample tick: t,front,left,back,right (all in metres). Run one trial per file.

Usage:
    ros2 run <pkg> event_recorder           # writes ge05_<timestamp>.csv
    python3 event_recorder.py my_trial.csv  # custom filename
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class EventRecorder(Node):
    def __init__(self, csv_path):
        super().__init__('event_recorder')

        # Latest reading per face; written out on each timer tick.
        self.latest = {'front': None, 'left': None, 'back': None, 'right': None}

        self.create_subscription(Range, '/crazyflie/range_front',
                                 lambda m: self._store('front', m), 10)
        self.create_subscription(Range, '/crazyflie/range_left',
                                 lambda m: self._store('left', m), 10)
        self.create_subscription(Range, '/crazyflie/range_back',
                                 lambda m: self._store('back', m), 10)
        self.create_subscription(Range, '/crazyflie/range_right',
                                 lambda m: self._store('right', m), 10)

        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t', 'front', 'left', 'back', 'right'])

        self.start_time = None
        self.rows = 0

        # Sample at 100 Hz to match the UDP bridge poll rate.
        self.create_timer(0.01, self._tick)
        self.get_logger().info(f"Recording to {csv_path}. Ctrl-C to stop.")

    def _store(self, face, msg):
        self.latest[face] = float(msg.range)

    def _tick(self):
        # Skip until every face has reported at least once.
        if any(v is None for v in self.latest.values()):
            return
        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time
        self.writer.writerow([
            f"{t:.4f}",
            f"{self.latest['front']:.4f}",
            f"{self.latest['left']:.4f}",
            f"{self.latest['back']:.4f}",
            f"{self.latest['right']:.4f}",
        ])
        self.rows += 1

    def close(self):
        self.csv_file.close()
        self.get_logger().info(f"Saved {self.rows} samples.")


def main(args=None):
    rclpy.init(args=args)
    csv_path = sys.argv[1] if len(sys.argv) > 1 else f"ge05_{int(time.time())}.csv"
    node = EventRecorder(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
