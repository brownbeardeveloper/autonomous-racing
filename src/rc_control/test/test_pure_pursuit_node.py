"""Tests for pure_pursuit_node."""

import time

from rc_control.pure_pursuit_node import PurePursuit


def test_a_control_step_without_tf_does_not_wait_for_it(ros):
    """Waiting stalls the whole node: the 50 Hz loop ran at 15 Hz with no map TF."""
    node = PurePursuit()
    sent = []
    node.pub.publish = sent.append
    try:
        durations = []
        for _ in range(5):
            start = time.monotonic()
            node.control_step()
            durations.append(time.monotonic() - start)
        assert min(durations) < 0.02
        assert sent[-1].drive.speed == 0.0
    finally:
        node.destroy_node()
