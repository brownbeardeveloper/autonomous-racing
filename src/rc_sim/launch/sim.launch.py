import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """The f1tenth gym on the spielberg track, configured by this package's config/sim.yaml.

    f1tenth_gym_ros's own launch file reads its config and its map from its
    own share directory, so this one starts the same nodes on ours instead.
    """
    gym_share = get_package_share_directory('f1tenth_gym_ros')
    # Without the extension: the gym loads map.yaml and the image next to it.
    track = os.path.join(get_package_share_directory('rc_maps'), 'tracks', 'spielberg', 'map')
    config = os.path.join(get_package_share_directory('rc_sim'), 'config', 'sim.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='f1tenth_gym_ros',
            executable='gym_bridge',
            name='bridge',
            output='screen',
            parameters=[config, {'map_path': track}],
        ),
        # The track as ground truth on /map, next to the map SLAM builds.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': track + '.yaml'}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_sim',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        # The gym car's model, hung on the TF the bridge publishes.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='ego_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', os.path.join(gym_share, 'launch', 'ego_racecar.xacro')]),
                value_type=str,
            )}],
            remappings=[('/robot_description', 'ego_robot_description')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(gym_share, 'launch', 'gym_bridge.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
