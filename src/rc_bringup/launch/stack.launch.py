import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Perception, control, safety and the monitor: the same on the car and in sim."""
    drive_mode = PythonExpression([
        "{'mapping': 'follow', 'racing': 'pure_pursuit', 'teleop': 'keyboard_teleop'}['",
        LaunchConfiguration('mode'), "']",
    ])

    def include(package, launch_file, **arguments):
        path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
        arguments = {
            'namespace': LaunchConfiguration('namespace'),
            'car_config': LaunchConfiguration('car_config'),
            **arguments,
        }
        return IncludeLaunchDescription(
            AnyLaunchDescriptionSource(path), launch_arguments=arguments.items()
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='rc_car',
            description="This car's ROS namespace; a second car takes another one",
        ),
        DeclareLaunchArgument(
            'car_config',
            default_value=os.path.join(
                get_package_share_directory('rc_bringup'), 'config', 'sim_car.yaml'
            ),
            description="Per-car parameters, layered on top of each package's defaults",
        ),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Track under rc_maps/tracks/ whose racingline.csv pure pursuit follows; '
                        'empty leaves it idle',
        ),
        DeclareLaunchArgument(
            'mode', default_value='mapping', choices=['mapping', 'racing', 'teleop'],
            description='Who drives - mapping: follow mode | racing: pure pursuit | teleop: keys',
        ),
        include('rc_perception', 'perception.launch.py'),
        include('rc_control', 'control.launch.py',
                map=LaunchConfiguration('map'), drive_mode=drive_mode),
        include('rc_safety', 'safety.launch.py'),
        include('rc_monitor', 'monitor.launch.py'),
    ])
