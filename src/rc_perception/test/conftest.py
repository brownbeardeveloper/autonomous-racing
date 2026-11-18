"""Shared fixtures, and a ROS_DOMAIN_ID of this package's own.

colcon runs the packages' test processes in parallel. The node tests use real
topics, so without a private domain they would see each other's traffic.
"""

import os

import pytest
import rclpy

ROS_DOMAIN_ID = '93'  # unique per rc_* package


def pytest_configure(config):
    os.environ['ROS_DOMAIN_ID'] = ROS_DOMAIN_ID  # before any rclpy.init()


@pytest.fixture(scope='session')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()
