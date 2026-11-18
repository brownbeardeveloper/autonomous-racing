#!/usr/bin/env python3
"""Keyboard drive candidate: WASD from the terminal onto keyboard_teleop/drive.

drive_mux_node relays it while its mode is keyboard_teleop. Run it in a
terminal of its own, with scripts/keyboard_teleop.sh: a node started by
`ros2 launch` gets no terminal on stdin and reads no keys.
"""

import select
import sys
import termios
import tty

from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rc_control.lib.keyboard_teleop import KeyboardDrive

RELEASE_TIMEOUT = 0.3  # [s] no key repeat within this window means the key was released
LIVE_TUNABLE = {'max_steering_angle', 'max_speed', 'steer_ramp_rate', 'throttle_ramp_rate'}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameters('', [
            ('drive_topic', 'keyboard_teleop/drive'),
            ('poll_rate', 50.0),            # [Hz]
            ('steer_ramp_rate', 3.0),       # [1/s] of full lock
            ('throttle_ramp_rate', 3.0),    # [1/s] of full throttle
            ('max_steering_angle', 0.36),   # [rad] at full lock
            ('max_speed', 2.0),             # [m/s] at full throttle
        ])
        p = self.get_parameter
        self.max_steering_angle = p('max_steering_angle').value
        self.max_speed = p('max_speed').value
        self.drive = KeyboardDrive(
            p('steer_ramp_rate').value, p('throttle_ramp_rate').value, RELEASE_TIMEOUT
        )

        self._term_fd = None
        if sys.stdin.isatty():
            self._term_fd = sys.stdin.fileno()
            self._term_settings = termios.tcgetattr(self._term_fd)
            tty.setcbreak(self._term_fd)  # keys arrive one by one, without Enter
        else:
            self.get_logger().warn('stdin is not a terminal - no keys will ever be read')

        self.pub = self.create_publisher(AckermannDriveStamped, p('drive_topic').value, 10)
        self._dt = 1.0 / p('poll_rate').value
        self.create_timer(self._dt, self._tick)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info('w/s throttle, a/d steering, space stop - hold to drive')

    def destroy_node(self):
        if self._term_fd is not None:
            termios.tcsetattr(self._term_fd, termios.TCSADRAIN, self._term_settings)
        super().destroy_node()

    def _tick(self) -> None:
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9
        while self._term_fd is not None and select.select([sys.stdin], [], [], 0.0)[0]:
            self.drive.key_pressed(sys.stdin.read(1), seconds)
        steer, throttle = self.drive.tick(seconds, self._dt)

        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.drive.steering_angle = steer * self.max_steering_angle
        msg.drive.speed = throttle * self.max_speed
        self.pub.publish(msg)

    def _on_parameter_update(self, params) -> SetParametersResult:
        for p in params:
            if p.name not in LIVE_TUNABLE:
                return SetParametersResult(
                    successful=False, reason=f"'{p.name}' can only be set at startup"
                )
        for p in params:
            # The ramp rates belong to the key state, the limits to this node.
            setattr(self.drive if p.name.endswith('ramp_rate') else self, p.name, p.value)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
