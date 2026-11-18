import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """The real car: lidar, laser odometry, the ESP32 bridge, the rc_* stack and Foxglove.

    mode:=mapping drives in follow mode while slam_toolbox builds a map; save
    it with scripts/save_map.sh. mode:=racing localizes on map:=<track>'s
    saved map and runs pure pursuit on its racingline.csv. mode:=teleop is
    driven from the keyboard, by scripts/keyboard_teleop.sh in a terminal.

    sim.launch.py has the same shape, with the sim bridge in place of the
    drivers. stack.launch.py is the part the two share.
    """
    # micro_ros_agent lives in ~/microros_ws, which this workspace does not
    # overlay. Node() would only miss it at execute time, with the lidar
    # already running, and that abort leaves the Hokuyo's socket held.
    try:
        get_package_share_directory('micro_ros_agent')
    except PackageNotFoundError:
        raise RuntimeError(
            'micro_ros_agent not found - source ~/microros_ws/install/setup.bash '
            'as well as this workspace (see README.md)'
        ) from None

    namespace = LaunchConfiguration('namespace')
    car_config = LaunchConfiguration('car_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def frame(name):
        """<namespace>/<name>: TF frames are not namespaced the way topics are."""
        return PythonExpression(["'", namespace, f"/{name}' if '", namespace, f"' else '{name}'"])

    def include(package, launch_file, condition=None, **arguments):
        path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
        return IncludeLaunchDescription(
            AnyLaunchDescriptionSource(path),
            launch_arguments=arguments.items(),
            condition=condition,
        )

    def require_map_for_racing(context):
        config = context.launch_configurations
        if config['mode'] == 'racing' and not config['map']:
            raise RuntimeError('mode:=racing needs map:=<track>, a folder under rc_maps/tracks/')

    # urg_node2 is a lifecycle node: started bare it stays unconfigured and
    # never opens the socket. The two handlers walk it to active, as its own
    # launch file does - which has no way to set frame_id, so it is not used.
    lidar = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2_node',
        namespace=namespace,
        output='screen',
        # car_config carries the field of view that keeps the car's own body out of the scan.
        parameters=[car_config, {
            'ip_address': LaunchConfiguration('lidar_ip_address'),
            # Declared as an int: a bare LaunchConfiguration would arrive as a string.
            'ip_port': ParameterValue(LaunchConfiguration('lidar_ip_port'), value_type=int),
            'frame_id': frame('laser'),
            'publish_intensity': False,
        }],
    )
    lidar_configure = RegisterEventHandler(OnProcessStart(
        target_action=lidar,
        on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))],
    ))
    lidar_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lidar,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))

    # The car has no wheel encoders, so rf2o is the only source of odom and of
    # the odom -> base_link TF. It matches the raw scan: a NaN beam that
    # scan_filtered fills in moves with the car and reads as "not moving".
    laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        namespace=namespace,
        output='screen',
        parameters=[{
            'laser_scan_topic': 'scan',
            'odom_topic': 'odom',
            'base_frame_id': frame('base_link'),
            'odom_frame_id': frame('odom'),
            'publish_tf': True,
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('laser_odometry')),
    )

    # Bridges the ESP32's serial link into ROS; the firmware subscribes to /drive.
    esp32_bridge = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        namespace=namespace,
        output='screen',
        arguments=[
            'serial',
            '--dev', LaunchConfiguration('esp32_serial_port'),
            '-b', LaunchConfiguration('esp32_baud_rate'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', choices=['mapping', 'racing', 'teleop'],
            description='mapping: follow mode + SLAM | racing: localization + pure pursuit | '
                        'teleop: the keyboard drives',
        ),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Track under rc_maps/tracks/: its map.yaml and racingline.csv. '
                        'Racing needs one; a mapping run is what creates it.',
        ),
        OpaqueFunction(function=require_map_for_racing),
        DeclareLaunchArgument(
            'namespace', default_value='rc_car',
            description="This car's ROS namespace; a second car takes another one",
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument(
            'car_config',
            default_value=os.path.join(
                get_package_share_directory('rc_bringup'), 'config', 'rc_car.yaml'
            ),
            description="Per-car parameters, layered on top of each package's defaults",
        ),
        DeclareLaunchArgument('lidar_ip_address', default_value='192.168.0.10'),
        DeclareLaunchArgument('lidar_ip_port', default_value='10940'),
        lidar,
        lidar_configure,
        lidar_activate,
        DeclareLaunchArgument(
            'laser_odometry', default_value='true',
            description='Run rf2o_laser_odometry, which makes odom from the lidar',
        ),
        laser_odometry,
        DeclareLaunchArgument('esp32_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('esp32_baud_rate', default_value='115200'),
        esp32_bridge,
        rviz,
        include('rc_description', 'description.launch.py',
                namespace=namespace, use_sim_time=use_sim_time),
        include('rc_bringup', 'stack.launch.py',
                namespace=namespace, car_config=car_config,
                map=LaunchConfiguration('map'), mode=LaunchConfiguration('mode')),
        include('rc_localization', 'mapping.launch.py',
                condition=LaunchConfigurationEquals('mode', 'mapping'),
                namespace=namespace, car_config=car_config, use_sim_time=use_sim_time),
        include('rc_localization', 'localization.launch.py',
                condition=LaunchConfigurationEquals('mode', 'racing'),
                namespace=namespace, car_config=car_config, use_sim_time=use_sim_time,
                map=PathJoinSubstitution([
                    get_package_share_directory('rc_maps'), 'tracks',
                    LaunchConfiguration('map'), 'map.yaml',
                ])),
        # Read-only: no publishing, parameters or services, so no Foxglove client
        # can clear the e-stop or turn a safety limit off.
        include('foxglove_bridge', 'foxglove_bridge_launch.xml',
                condition=IfCondition(LaunchConfiguration('foxglove')),
                port=LaunchConfiguration('foxglove_port'),
                capabilities='[connectionGraph,assets]'),
    ])
