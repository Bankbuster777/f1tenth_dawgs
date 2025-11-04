#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for sector tuner"""

    # Package directory
    pkg_dir = get_package_share_directory('path_sector_editor')

    # Launch arguments
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='',
        description='Path to CSV waypoint file (x,y,v,kappa)'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('path_sector_editor'),
            'config',
            'path_sector_editor.yaml'
        ]),
        description='Path to configuration file'
    )

    # Path sector editor node
    path_sector_editor_node = Node(
        package='path_sector_editor',
        executable='path_sector_editor_node',
        name='path_sector_editor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'csv_file_path': LaunchConfiguration('csv_file')
            }
        ]
    )

    return LaunchDescription([
        csv_file_arg,
        config_file_arg,
        path_sector_editor_node,
    ])
