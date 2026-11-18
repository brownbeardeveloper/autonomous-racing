#!/usr/bin/env python3
"""Clean the raw lidar scan for follow_node. rf2o, SLAM and safety_node read the raw scan.

NaN and inf become range_max: urg_node2 publishes NaN for "no echo" (open
space, glass, a black surface), which means no hit. Everything is clamped to
the scan's own [range_min, range_max], so a reading below range_min stays
"very close".
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter_node')
        self.declare_parameters('', [('scan_topic', 'scan'), ('filtered_topic', 'scan_filtered')])
        scan_topic = self.get_parameter('scan_topic').value
        filtered_topic = self.get_parameter('filtered_topic').value

        self.pub = self.create_publisher(LaserScan, filtered_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.get_logger().info(f'filtering {scan_topic} -> {filtered_topic}')

    def filter(self, scan: LaserScan) -> np.ndarray:
        """Ranges with NaN and inf as range_max, clamped to the scan's own bounds."""
        r = np.asarray(scan.ranges, dtype=np.float32)
        r = np.nan_to_num(r, nan=scan.range_max, posinf=scan.range_max, neginf=0.0)
        return np.clip(r, scan.range_min, scan.range_max)

    def scan_cb(self, scan: LaserScan) -> None:
        scan.ranges = self.filter(scan).tolist()
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
