import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # The namespace goes into the URDF's link names through xacro, not into
    # robot_state_publisher's frame_prefix: the names it serves on
    # robot_description then match the TF frames it publishes, which is what
    # Foxglove and RViz attach the model by.
    prefix = PythonExpression(["'", namespace, "/' if '", namespace, "' else ''"])
    # value_type=str: the xacro output is XML, which would otherwise be parsed as YAML.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' prefix:=', prefix]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(
                get_package_share_directory('rc_description'), 'urdf', 'rc_car.urdf.xacro'
            ),
            description="The car's URDF or xacro file",
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_joint_state_publisher', default_value='true',
            description='Publish zeros for the steering joints, which nothing actuates yet',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}, use_sim_time],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
        ),
    ])
