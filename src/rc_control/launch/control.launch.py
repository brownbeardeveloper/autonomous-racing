import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('rc_control'), 'config')
    namespace = LaunchConfiguration('namespace')
    parameters = [os.path.join(config_dir, 'basic.yaml'), LaunchConfiguration('car_config')]

    def node(executable, *overrides):
        return Node(
            package='rc_control',
            executable=executable,
            name=executable,
            namespace=namespace,
            output='screen',
            parameters=[*parameters, *overrides],
        )

    def pure_pursuit_node(context):
        # TF frames are not namespaced the way topics are, so the frame is
        # built here. Set last: a base_frame in the YAML has no effect.
        ns = namespace.perform(context)
        overrides = {'base_frame': f'{ns}/rear_axle' if ns else 'rear_axle'}
        track = LaunchConfiguration('map').perform(context)
        if track:
            overrides['waypoints_file'] = os.path.join(
                get_package_share_directory('rc_maps'), 'tracks', track, 'racingline.csv'
            )
        return [node('pure_pursuit_node', overrides)]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        # rc_bringup's per-car overrides, loaded after basic.yaml so they win.
        DeclareLaunchArgument('car_config', default_value=os.path.join(config_dir, 'empty.yaml')),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Track under rc_maps/tracks/ whose racingline.csv pure pursuit follows; '
                        'empty leaves it idle',
        ),
        DeclareLaunchArgument(
            'drive_mode', default_value='follow',
            choices=['follow', 'pure_pursuit', 'keyboard_teleop'],
            description="drive_mux_node's start mode",
        ),
        OpaqueFunction(function=pure_pursuit_node),
        node('follow_node'),
        node('drive_mux_node', {'mode': LaunchConfiguration('drive_mode')}),
    ])
