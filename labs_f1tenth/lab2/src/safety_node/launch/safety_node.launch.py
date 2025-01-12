from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    safety_config = os.path.join(
        get_package_share_directory('safety_node'),
        'config',
        'safety_node.yaml'
    )

    safety_la = DeclareLaunchArgument(
        'safety_config',
        default_value=safety_config,
        description='Configs for safety node on sim and dawgs'
    )

    ld = LaunchDescription([safety_la])

    safety_nd = Node(
        package='safety_node',
        executable='safety_node',
        name='safety_node',
        parameters=[LaunchConfiguration('safety_config')]
    )

    ld.add_action(safety_nd)

    return ld
