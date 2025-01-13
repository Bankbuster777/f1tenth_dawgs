import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    parameter_config = os.path.join(
        get_package_share_directory('lab1_introduction'),
        'config',
        'param.yaml'
    )

    param_launch_argument = DeclareLaunchArgument(
        'param_config',
        default_value=parameter_config,
        description='parameters for related packages'
    )

    launch_description = LaunchDescription([param_launch_argument])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('param_config')]
    )

    ackermann_talker_node = Node(
        package='lab1_introduction',
        executable='ackermann_talker',
        name='ackermann_talker',
        parameters=[LaunchConfiguration('param_config')]
    )

    launch_description.add_action(joy_node)
    launch_description.add_action(ackermann_talker_node)
    return launch_description
