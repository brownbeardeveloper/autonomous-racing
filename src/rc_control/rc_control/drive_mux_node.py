#!/usr/bin/env python3
"""Relay the active mode's drive command onto drive_cmd.

Every candidate controller (follow, pure_pursuit, keyboard_teleop) publishes on
a topic of its own; only the active mode's messages are passed on, unchanged.
The launch file sets the start mode. `ros2 param set <node> mode <name>` or a
message on behavior_mode switches it, and the most recent of the two wins.
"""

from functools import partial

from ackermann_msgs.msg import AckermannDriveStamped
from rc_msgs.msg import BehaviorMode
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def resolve_mode(requested: str, valid_modes, default: str) -> str:
    """Return `requested` if it is a known mode, else `default`."""
    return requested if requested in valid_modes else default


class DriveMux(Node):
    def __init__(self):
        super().__init__('drive_mux_node')

        self.declare_parameters('', [
            ('mode', 'follow'),
            ('drive_cmd_topic', 'drive_cmd'),
            ('behavior_mode_topic', 'behavior_mode'),
            ('follow_drive_topic', 'follow/drive'),
            ('pure_pursuit_drive_topic', 'pure_pursuit/drive'),
            ('keyboard_teleop_drive_topic', 'keyboard_teleop/drive'),
        ])
        p = self.get_parameter
        self._mode_topics = {
            'follow': p('follow_drive_topic').value,
            'pure_pursuit': p('pure_pursuit_drive_topic').value,
            'keyboard_teleop': p('keyboard_teleop_drive_topic').value,
        }
        requested = p('mode').value
        self.active_mode = resolve_mode(requested, self._mode_topics, 'follow')
        if self.active_mode != requested:
            self.get_logger().warn(f"unknown mode '{requested}', starting in '{self.active_mode}'")

        self.pub = self.create_publisher(AckermannDriveStamped, p('drive_cmd_topic').value, 10)
        for mode, topic in self._mode_topics.items():
            self.create_subscription(
                AckermannDriveStamped, topic, partial(self.candidate_cb, mode=mode), 10
            )
        # Transient local: a mux that starts late still gets the mode published before it.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            BehaviorMode, p('behavior_mode_topic').value, self.behavior_mode_cb, latched
        )
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(f'mode={self.active_mode}, candidates={self._mode_topics}')

    def candidate_cb(self, msg: AckermannDriveStamped, mode: str) -> None:
        if mode == self.active_mode:
            self.pub.publish(msg)

    def behavior_mode_cb(self, msg: BehaviorMode) -> None:
        if msg.mode in self._mode_topics:
            self._switch(msg.mode)
        else:
            self.get_logger().warn(f"unknown mode '{msg.mode}' on behavior_mode, ignoring")

    def _switch(self, mode: str) -> None:
        if mode != self.active_mode:
            self.active_mode = mode
            self.get_logger().info(f'switched to mode={mode}')

    def _on_parameter_update(self, params) -> SetParametersResult:
        for p in params:
            if p.name != 'mode':
                return SetParametersResult(
                    successful=False, reason=f"'{p.name}' can only be set at startup"
                )
            if p.value not in self._mode_topics:
                return SetParametersResult(
                    successful=False,
                    reason=f"unknown mode '{p.value}', must be one of {list(self._mode_topics)}",
                )
        for p in params:
            self._switch(p.value)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = DriveMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
