#!/usr/bin/env python3
"""
Launch file for offline path analysis

Analyzes recorded trajectory CSV files and compares them with global centerline CSV file.
Displays trajectories on map image overlay.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for offline analysis."""

    # Declare launch arguments
    global_path_csv_arg = DeclareLaunchArgument(
        'global_path_csv',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1030/mohyun_slam_v3_iqp.csv',
        description='Path to global centerline CSV file'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1030/mohyun_slam_v3.yaml',
        description='Path to map YAML file (for overlay visualization)'
    )

    csv_directory_arg = DeclareLaunchArgument(
        'csv_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/utilities/path_recorder/data',
        description='Directory containing CSV recording files'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='',
        description='Specific CSV file to analyze (optional, defaults to most recent)'
    )

    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/utilities/path_recorder/data',
        description='Directory to save analysis plots'
    )

    save_plots_arg = DeclareLaunchArgument(
        'save_plots',
        default_value='true',
        description='Save plots to file'
    )

    show_plots_arg = DeclareLaunchArgument(
        'show_plots',
        default_value='true',
        description='Show plots in GUI'
    )

    # Offline analysis node
    offline_analysis_node = Node(
        package='path_recorder',
        executable='offline_analysis_node',
        name='offline_analysis_node',
        output='screen',
        parameters=[
            {
                'global_path_csv': LaunchConfiguration('global_path_csv'),
                'map_yaml': LaunchConfiguration('map_yaml'),
                'csv_directory': LaunchConfiguration('csv_directory'),
                'csv_file': LaunchConfiguration('csv_file'),
                'output_directory': LaunchConfiguration('output_directory'),
                'save_plots': LaunchConfiguration('save_plots'),
                'show_plots': LaunchConfiguration('show_plots'),
            }
        ]
    )

    return LaunchDescription([
        global_path_csv_arg,
        map_yaml_arg,
        csv_directory_arg,
        csv_file_arg,
        output_directory_arg,
        save_plots_arg,
        show_plots_arg,
        offline_analysis_node,
    ])
