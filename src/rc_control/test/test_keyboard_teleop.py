"""Tests for keyboard teleop: the key state on its own, then the node on its topic."""

from ackermann_msgs.msg import AckermannDriveStamped
import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rc_control.keyboard_teleop_node import KeyboardTeleop
from rc_control.lib.keyboard_teleop import KeyboardDrive


def new_drive():
    return KeyboardDrive(steer_ramp_rate=3.0, throttle_ramp_rate=3.0, release_timeout=0.3)


def test_throttle_ramps_rather_than_snaps():
    drive = new_drive()
    drive.key_pressed('w', now=0.0)
    _, throttle = drive.tick(now=0.0, dt=0.02)
    assert 0.0 < throttle < 1.0
    assert throttle == pytest.approx(0.06)


def test_throttle_reverses_on_s():
    drive = new_drive()
    drive.key_pressed('s', now=0.0)
    _, throttle = drive.tick(now=0.0, dt=0.02)
    assert throttle < 0.0


def test_throttle_reaches_full_after_enough_ticks():
    drive = new_drive()
    drive.key_pressed('w', now=0.0)
    throttle = 0.0
    for _ in range(50):
        _, throttle = drive.tick(now=0.0, dt=0.02)
    assert throttle == pytest.approx(1.0)


def test_throttle_stops_immediately_after_release_timeout():
    drive = new_drive()
    drive.key_pressed('w', now=0.0)
    for _ in range(50):
        drive.tick(now=0.0, dt=0.02)
    _, throttle = drive.tick(now=0.5, dt=0.02)
    assert throttle == pytest.approx(0.0)


def test_throttle_stays_stopped_on_the_next_tick_too():
    drive = new_drive()
    drive.key_pressed('w', now=0.0)
    for _ in range(50):
        drive.tick(now=0.0, dt=0.02)
    drive.tick(now=0.5, dt=0.02)
    _, throttle = drive.tick(now=0.52, dt=0.02)
    assert throttle == pytest.approx(0.0)


def test_steer_ramps_rather_than_snaps():
    drive = new_drive()
    drive.key_pressed('a', now=0.0)
    steer, _ = drive.tick(now=0.0, dt=0.02)
    assert 0.0 < steer < 1.0
    assert steer == pytest.approx(0.06)


def test_d_key_steers_right_which_is_negative():
    drive = new_drive()
    drive.key_pressed('d', now=0.0)
    steer, _ = drive.tick(now=0.0, dt=0.02)
    assert steer < 0.0


def test_steer_reaches_full_lock_after_enough_ticks():
    drive = new_drive()
    drive.key_pressed('a', now=0.0)
    steer = 0.0
    for _ in range(50):
        steer, _ = drive.tick(now=0.0, dt=0.02)
    assert steer == pytest.approx(1.0)


def test_steer_ramps_back_to_zero_on_release():
    drive = new_drive()
    drive.key_pressed('a', now=0.0)
    for _ in range(50):
        drive.tick(now=0.0, dt=0.02)
    steer, _ = drive.tick(now=0.5, dt=0.02)
    assert steer < 1.0


def test_space_stops_immediately():
    drive = new_drive()
    drive.key_pressed('a', now=0.0)
    drive.key_pressed('w', now=0.0)
    for _ in range(50):
        drive.tick(now=0.0, dt=0.02)
    drive.key_pressed(' ', now=0.0)
    steer, throttle = drive.tick(now=0.0, dt=0.02)
    assert (steer, throttle) == (0.0, 0.0)


def test_space_does_not_leave_a_stale_throttle_target():
    drive = new_drive()
    drive.key_pressed('w', now=0.0)
    for _ in range(50):
        drive.tick(now=0.0, dt=0.02)
    drive.key_pressed(' ', now=0.0)
    drive.tick(now=0.0, dt=0.02)
    _, throttle = drive.tick(now=0.02, dt=0.02)
    assert throttle == pytest.approx(0.0)


@pytest.fixture
def node(ros):
    n = KeyboardTeleop()
    yield n
    n.destroy_node()


class Harness(Node):
    """Listens on keyboard_teleop/drive."""

    def __init__(self):
        super().__init__('keyboard_teleop_harness')
        self.drives = []
        self.create_subscription(
            AckermannDriveStamped, '/keyboard_teleop/drive', self.drives.append, 10
        )


@pytest.fixture
def rig(node):
    harness = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(harness)
    yield node, harness, ex
    ex.remove_node(node)
    ex.remove_node(harness)
    ex.shutdown()
    harness.destroy_node()


def test_node_publishes_on_drive_topic(rig):
    """A test process has no tty, so no key is read: this proves the node comes up."""
    _, harness, ex = rig
    for _ in range(20):
        ex.spin_once(timeout_sec=0.05)
        if harness.drives:
            break
    assert harness.drives, 'node did not publish within the timeout'
    assert harness.drives[-1].drive.speed == pytest.approx(0.0)
    assert harness.drives[-1].drive.steering_angle == pytest.approx(0.0)
