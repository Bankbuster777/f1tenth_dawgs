from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    wall_follow_pid_config = os.path.join(
        get_package_share_directory('wall_follow'),
        'config',
        'wall_follow_pid.yaml'
    )

    wall_pid_la = DeclareLaunchArgument(
        'wall_follow_pid_config',
        default_value=wall_follow_pid_config,
        description='Configs for wall follow node on PID tuning values'
    )

    ld = LaunchDescription([wall_pid_la])

    wall_follow_node = Node(
        package='wall_follow',
        executable='wall_follow_node',
        name='wall_follow_node',
        parameters=[LaunchConfiguration('wall_follow_pid_config')]
    )

    ld.add_action(wall_follow_node)

    return ld
