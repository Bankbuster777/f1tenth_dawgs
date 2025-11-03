#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_bringup = LaunchConfiguration('use_bringup', default='true')
    use_ekf = LaunchConfiguration('use_ekf', default='true')

    agent_dawgs_prefix = get_package_share_directory('agent_dawgs')
    f1tenth_stack_prefix = get_package_share_directory('f1tenth_stack')

    particle_filter_config = LaunchConfiguration('particle_filter_config',
        default=os.path.join(
            get_package_share_directory('agent_dawgs'),
            'config',
            'pf_localize.yaml'
        ))

    rviz_config_dir = os.path.join(agent_dawgs_prefix, 'rviz', 'particle_localization.rviz')

    # F1TENTH bringup launch (without static map->odom for localization)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(f1tenth_stack_prefix, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={'publish_map_odom_tf': 'false'}.items(),
        condition=IfCondition(use_bringup)
    )

    # EKF launch for odometry/filtered
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(agent_dawgs_prefix, 'launch', 'ekf_launch.py')
        ]),
        condition=IfCondition(use_ekf)
    )

    # Map file configuration
    yaml_file = LaunchConfiguration('yaml_file',
        default='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun_1030/mohyun_slam_v3.yaml'),

    return LaunchDescription([
        DeclareLaunchArgument(
            'particle_filter_config',
            default_value=particle_filter_config,
            description='Full path to particle filter config file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'),
        DeclareLaunchArgument(
            'use_bringup',
            default_value='true',
            description='Launch F1TENTH hardware bringup'),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Launch EKF for odometry filtering'),

        # F1TENTH hardware bringup
        bringup_launch,

        # EKF for odometry filtering
        ekf_launch,

        # Map server node (lifecycle managed)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': yaml_file,
            }],
            output='screen'),

        # Lifecycle manager for map server - starts map_server automatically
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # Particle filter node - started after map server through respawn delay
        Node(
            package='particle_filter',
            executable='particle_filter',
            name='particle_filter',
            output='screen',
            parameters=[
                particle_filter_config,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odom', 'odometry/filtered'),
            ],
            # Add respawn to handle startup race condition
            respawn=True,
            respawn_delay=3.0,
        ),

        # # InitialPose bridge for RViz 2D Pose Estimate
        # Node(
        #     package='agent_dawgs',
        #     executable='initialpose_bridge.py',
        #     name='initialpose_bridge',
        #     output='screen'),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
