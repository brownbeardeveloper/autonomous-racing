#!/usr/bin/env python3
"""Record a driven lap as a waypoints CSV that pure_pursuit_node can load.

Keeps a point every --min-spacing metres, read from TF --frame -> --base-frame:
map -> base_link by default, the frame the saved map and pure pursuit share.
Not the odometry pose: rf2o's odom frame drifts away from the map. The speed
comes from the odom twist. Ctrl+C writes a plain "x,y,v" CSV, without a header.

    source install/setup.bash
    python3 scripts/record_waypoints.py --output src/rc_maps/tracks/<name>/racingline.csv
    # sim: --base-frame ego_racecar/base_link --odom-topic /ego_racecar/odom

Drive one lap and stop near the start. The last point need not meet the first:
pure pursuit closes the loop itself.
"""

import argparse
import math
import sys

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class WaypointRecorder(Node):
    def __init__(self, args):
        super().__init__('waypoint_recorder')
        self.frame = args.frame
        self.base_frame = args.base_frame
        self.min_spacing = args.min_spacing
        self.default_speed = args.default_speed
        self.speed = 0.0
        self.points = []  # (x, y, v)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.create_timer(0.05, self.record)
        self.get_logger().info(
            f'recording {self.frame} -> {self.base_frame}, a point every {self.min_spacing} m'
        )

    def odom_cb(self, msg: Odometry) -> None:
        self.speed = msg.twist.twist.linear.x

    def record(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'waiting for TF: {e}', throttle_duration_sec=2.0)
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        if self.points:
            last_x, last_y, _ = self.points[-1]
            if math.hypot(x - last_x, y - last_y) < self.min_spacing:
                return
        # Standing at the start line the twist reads about 0: not a speed to race at.
        v = self.speed if self.speed > 0.1 else self.default_speed
        self.points.append((x, y, v))
        if len(self.points) % 10 == 0:
            self.get_logger().info(f'{len(self.points)} waypoints recorded')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    arg = parser.add_argument
    arg('--output', required=True, help='CSV file to write on exit')
    arg('--frame', default='map', help='frame the waypoints are written in')
    arg('--base-frame', default='rc_car/base_link', help="the car's rear-axle frame")
    arg('--odom-topic', default='/rc_car/odom', help='source of the recorded speed')
    arg('--min-spacing', type=float, default=0.15, help='metres between recorded points')
    arg('--default-speed', type=float, default=2.0, help='speed written while the twist reads ~0')
    args = parser.parse_args()

    rclpy.init()
    node = WaypointRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.points:
            with open(args.output, 'w') as f:
                f.writelines(f'{x:.4f},{y:.4f},{v:.3f}\n' for x, y, v in node.points)
            print(f'wrote {len(node.points)} waypoints to {args.output}')
        else:
            print('no waypoints recorded - nothing written', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
