#!/usr/bin/env python3
"""UDP heartbeat from scripts/estop_remote.py -> estop (std_msgs/Bool, true = STOP)."""

import json
import socket
import time

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from rc_safety.estop_state import EStopState, parse_packet, read_token


class EStopBridge(Node):
    def __init__(self):
        super().__init__('estop_bridge_node')
        # Read-only, as safety_node's: set at startup, never changed on a running car.
        self.declare_parameters('', [
            (name, value, ParameterDescriptor(read_only=True)) for name, value in [
                ('udp_port', 5005),
                ('timeout_s', 0.5),
                ('token_file', '~/.config/rc_car/estop_token'),
            ]
        ])
        token_file = self.get_parameter('token_file').value
        try:
            self.token = read_token(token_file) if token_file else ''
        except (OSError, ValueError) as e:
            # Exiting is the safe failure: safety_node brakes on a silent estop.
            self.get_logger().fatal(f'no e-stop token: {e} - see README, Remote e-stop')
            raise SystemExit(1)
        if not self.token:
            self.get_logger().warn('no token_file: any device on the network can reset the e-stop')

        self.state = EStopState(self.get_parameter('timeout_s').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.get_parameter('udp_port').value))
        self.sock.setblocking(False)
        self._remote = None
        self._logged = None

        self.pub = self.create_publisher(Bool, 'estop', 10)
        self.create_timer(0.05, self._tick)

    def _tick(self) -> None:
        now = time.monotonic()  # the link timeout is wall time, even in sim
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except OSError:
                break
            pkt = parse_packet(data, self.token)
            if pkt:
                self._remote = addr
                self.state.on_packet(pkt, now)

        stopped = self.state.active(now)
        self.pub.publish(Bool(data=stopped))

        if self.state.reason != self._logged:
            self._logged = self.state.reason
            # Two calls: rclpy rejects two severities from one call site.
            if stopped:
                self.get_logger().warn(f'e-stop: {self.state.reason}')
            else:
                self.get_logger().info('e-stop released')

        if self._remote:
            reply = json.dumps({'stopped': stopped, 'reason': self.state.reason})
            try:
                self.sock.sendto(reply.encode(), self._remote)
            except OSError:
                pass  # WiFi down; must not kill the node


def main(args=None):
    rclpy.init(args=args)
    node = EStopBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
