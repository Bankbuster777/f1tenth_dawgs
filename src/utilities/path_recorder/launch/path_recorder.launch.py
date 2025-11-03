#!/usr/bin/env python3
"""
Launch file for path_recorder node

Launches the path recorder to record vehicle trajectory and calculate lateral errors.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for path recorder."""

    # Declare launch arguments
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='Odometry topic to record'
    )

    global_path_topic_arg = DeclareLaunchArgument(
        'global_path_topic',
        default_value='/global_centerline',
        description='Global path topic for comparison'
    )

    recording_enabled_arg = DeclareLaunchArgument(
        'recording_enabled',
        default_value='true',
        description='Enable/disable recording'
    )

    save_to_csv_arg = DeclareLaunchArgument(
        'save_to_csv',
        default_value='true',
        description='Save recording data to CSV file'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('path_recorder'),
        'config',
        'recorder_params.yaml'
    ])

    # Path recorder node
    path_recorder_node = Node(
        package='path_recorder',
        executable='path_recorder_node',
        name='path_recorder_node',
        output='screen',
        parameters=[
            config_file,
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'global_path_topic': LaunchConfiguration('global_path_topic'),
                'recording_enabled': LaunchConfiguration('recording_enabled'),
                'save_to_csv': LaunchConfiguration('save_to_csv'),
            }
        ],
        remappings=[
            # Add any remappings here if needed
        ]
    )

    return LaunchDescription([
        odom_topic_arg,
        global_path_topic_arg,
        recording_enabled_arg,
        save_to_csv_arg,
        path_recorder_node,
    ])
