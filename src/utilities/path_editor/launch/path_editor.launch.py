from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare arguments
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/songdo_1104/songdo_slam_ekf_iqp.csv',
        description='Path to racing path CSV file'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for published path'
    )

    map_image_arg = DeclareLaunchArgument(
        'map_image',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/songdo_1104/songdo_slam_ekf.pgm',
        description='Path to map image file (pgm/png)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/songdo_1104/songdo_slam_ekf.yaml',
        description='Path to map YAML file with origin/resolution info'
    )

    # Path editor node
    path_editor_node = Node(
        package='path_editor',
        executable='path_editor_node.py',
        name='path_editor',
        output='screen',
        parameters=[{
            'csv_file': LaunchConfiguration('csv_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'map_image': LaunchConfiguration('map_image'),
            'map_yaml': LaunchConfiguration('map_yaml')
        }]
    )

    return LaunchDescription([
        csv_file_arg,
        frame_id_arg,
        map_image_arg,
        map_yaml_arg,
        path_editor_node,
    ])
