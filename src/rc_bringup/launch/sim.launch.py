import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """The simulated car: the f1tenth gym bridge, the rc_* stack and Foxglove.

    mode:=mapping drives in follow mode while slam_toolbox builds a map.
    mode:=racing runs pure pursuit on map:=<track>'s racingline.csv, posed by
    the gym's ground truth. mode:=teleop is driven from the keyboard, by
    scripts/keyboard_teleop.sh in a terminal.

    Everything runs on the wall clock, as the gym does: it publishes no /clock.

    car.launch.py has the same shape, with the real drivers in place of the
    bridge. stack.launch.py is the part the two share.
    """
    namespace = LaunchConfiguration('namespace')
    car_config = LaunchConfiguration('car_config')

    def include(package, launch_file, condition=None, **arguments):
        path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
        return IncludeLaunchDescription(
            AnyLaunchDescriptionSource(path),
            launch_arguments=arguments.items(),
            condition=condition,
        )

    # Both base_links are the rear axle. This hangs our car's TF tree, and its
    # model, on the gym's ground-truth pose.
    attach_to_gym = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='attach_to_gym',
        arguments=[
            '--frame-id', 'ego_racecar/base_link',
            '--child-frame-id', PythonExpression(
                ["'", namespace, "/base_link' if '", namespace, "' else 'base_link'"]
            ),
        ],
    )

    scan_relay = Node(
        package='rc_sim',
        executable='scan_relay_node',
        name='scan_relay_node',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', choices=['mapping', 'racing', 'teleop'],
            description='mapping: follow mode + SLAM | racing: pure pursuit on map:=<track> | '
                        'teleop: the keyboard drives',
        ),
        DeclareLaunchArgument(
            'namespace', default_value='rc_car',
            description="This car's ROS namespace; a second car takes another one",
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument(
            'car_config',
            default_value=os.path.join(
                get_package_share_directory('rc_bringup'), 'config', 'sim_car.yaml'
            ),
            description="Per-car parameters, layered on top of each package's defaults",
        ),
        DeclareLaunchArgument(
            'map', default_value='spielberg',
            description='Track under rc_maps/tracks/ whose racingline.csv pure pursuit follows; '
                        "the default is the gym's own track",
        ),
        include('rc_sim', 'sim.launch.py', rviz=LaunchConfiguration('rviz')),
        # Our own URDF, next to the ego_racecar model the gym bridge publishes.
        include('rc_description', 'description.launch.py', namespace=namespace),
        attach_to_gym,
        scan_relay,
        include('rc_bringup', 'stack.launch.py',
                namespace=namespace, car_config=car_config,
                map=LaunchConfiguration('map'), mode=LaunchConfiguration('mode')),
        # The gym publishes ground truth as map -> ego_racecar/base_link. That
        # is the odometry here, so SLAM's own map frame needs another name.
        include('rc_localization', 'mapping.launch.py',
                condition=LaunchConfigurationEquals('mode', 'mapping'),
                namespace=namespace, car_config=car_config,
                map_frame='slam_map', odom_frame='map', base_frame='ego_racecar/base_link'),
        # Read-only, as on the car: see car.launch.py.
        include('foxglove_bridge', 'foxglove_bridge_launch.xml',
                condition=IfCondition(LaunchConfiguration('foxglove')),
                port=LaunchConfiguration('foxglove_port'),
                capabilities='[connectionGraph,assets]'),
    ])
