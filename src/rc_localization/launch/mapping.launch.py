import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Online SLAM with slam_toolbox: builds the track map while the car drives.

    The live map is on <namespace>/map; scripts/save_map.sh writes it to
    rc_maps/tracks/<track>/, which localization.launch.py loads for racing.
    slam_toolbox publishes map -> odom itself, so the two never run together.

    The frames are launch arguments rather than car_config keys: a parameter
    set here by node name always beats a /** YAML entry, whatever the order.
    """
    config_dir = os.path.join(get_package_share_directory('rc_localization'), 'config')
    namespace = LaunchConfiguration('namespace')

    def frame(name):
        """<namespace>/<name>: TF frames are not namespaced the way topics are."""
        return PythonExpression(["'", namespace, f"/{name}' if '", namespace, f"' else '{name}'"])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value=frame('odom')),
        DeclareLaunchArgument('base_frame', default_value=frame('base_link')),
        DeclareLaunchArgument('car_config', default_value=os.path.join(config_dir, 'empty.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            output='screen',
            parameters=[
                os.path.join(config_dir, 'slam_toolbox.yaml'),
                LaunchConfiguration('car_config'),
                {
                    'map_frame': LaunchConfiguration('map_frame'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
