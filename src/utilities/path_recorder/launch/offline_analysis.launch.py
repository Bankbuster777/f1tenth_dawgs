#!/usr/bin/env python3
"""
Launch file for offline path analysis

Analyzes recorded trajectory CSV files and compares them with global centerline CSV file.
Displays trajectories on map image overlay.

Configuration is loaded from config/recorder_params.yaml by default.
You can override settings via launch arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for offline analysis."""

    # Get package share directory
    pkg_share = FindPackageShare('path_recorder')

    # Default config file path
    default_config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'recorder_params.yaml'
    ])

    # Declare launch arguments (these override yaml settings if provided)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to recorder_params.yaml config file'
    )

    global_path_csv_arg = DeclareLaunchArgument(
        'global_path_csv',
        default_value='',
        description='Path to global centerline CSV file (overrides yaml if provided)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Path to map YAML file (overrides yaml if provided)'
    )

    csv_directory_arg = DeclareLaunchArgument(
        'csv_directory',
        default_value='',
        description='Directory containing CSV recording files (overrides yaml if provided)'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='',
        description='Specific CSV file to analyze (overrides yaml if provided)'
    )

    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='',
        description='Directory to save analysis plots (overrides yaml if provided)'
    )

    save_plots_arg = DeclareLaunchArgument(
        'save_plots',
        default_value='',
        description='Save plots to file (overrides yaml if provided)'
    )

    show_plots_arg = DeclareLaunchArgument(
        'show_plots',
        default_value='',
        description='Show plots in GUI (overrides yaml if provided)'
    )

    # Offline analysis node
    offline_analysis_node = Node(
        package='path_recorder',
        executable='offline_analysis_node',
        name='offline_analysis_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # Launch arguments override yaml settings when provided (non-empty)
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
        config_file_arg,
        global_path_csv_arg,
        map_yaml_arg,
        csv_directory_arg,
        csv_file_arg,
        output_directory_arg,
        save_plots_arg,
        show_plots_arg,
        offline_analysis_node,
    ])
