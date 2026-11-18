import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('rc_perception'), 'config')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        # rc_bringup's per-car overrides, loaded after basic.yaml so they win.
        DeclareLaunchArgument('car_config', default_value=os.path.join(config_dir, 'empty.yaml')),
        Node(
            package='rc_perception',
            executable='scan_filter_node',
            name='scan_filter_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[os.path.join(config_dir, 'basic.yaml'), LaunchConfiguration('car_config')],
        ),
    ])
