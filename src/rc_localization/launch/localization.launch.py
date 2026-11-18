import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Localization on a saved map: map_server, the particle filter and map -> odom.

    The particle filter replaces nav2_amcl, whose sensor model was too slow at
    this car's speed. Only map_server is a lifecycle node.
    """
    config_dir = os.path.join(get_package_share_directory('rc_localization'), 'config')
    config = os.path.join(config_dir, 'particle_filter.yaml')
    car_config = LaunchConfiguration('car_config')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    def frame(name):
        """<namespace>/<name>: TF frames are not namespaced the way topics are."""
        return PythonExpression(["'", namespace, f"/{name}' if '", namespace, f"' else '{name}'"])

    # 'map' carries no prefix: it is the one frame every car shares.
    frames = {'map_frame': 'map', 'odom_frame': frame('odom'), 'base_frame': frame('base_link')}

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        # rc_bringup's per-car overrides, loaded after particle_filter.yaml so they win.
        DeclareLaunchArgument('car_config', default_value=os.path.join(config_dir, 'empty.yaml')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('rc_maps'), 'tracks', 'spielberg', 'map.yaml'
            ),
            description='Map YAML file to localize against',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[
                config, car_config, {'yaml_filename': LaunchConfiguration('map')}, use_sim_time
            ],
        ),
        # Takes map_server through configure and activate.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=namespace,
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}, use_sim_time],
        ),
        Node(
            package='rc_localization',
            executable='particle_filter_node',
            name='particle_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[
                config, car_config, use_sim_time,
                {'map_frame': frames['map_frame'], 'base_frame': frames['base_frame']},
            ],
        ),
        Node(
            package='rc_localization',
            executable='map_odom_broadcaster_node',
            name='map_odom_broadcaster_node',
            namespace=namespace,
            output='screen',
            parameters=[frames, use_sim_time],
        ),
    ])
