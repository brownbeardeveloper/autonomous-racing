#!/usr/bin/env python3
"""Republish the gym's /scan as this car's `scan`, with a correct header.

f1tenth_gym_ros stamps angle_increment = fov / beams, but the gym spreads its
beams over the whole fov: fov / (beams - 1). slam_toolbox counts the beams
from the header and rejects every scan that disagrees ("1080 range readings,
expected 1081"). The relay also gives the sim the same `scan` topic as the car.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay_node')
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.relay, qos_profile_sensor_data)

    def relay(self, scan: LaserScan) -> None:
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(scan.ranges) - 1)
        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
