import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('M3Pro_robot_navigation')

    scan_merger_params = os.path.join(pkg_share, 'config', 'scan_merger.yaml')
    mapper_params = os.path.join(pkg_share, 'config', 'mapper_params.yaml')

    merger_node = Node(
        package='M3Pro_robot_navigation',
        executable='multi_lidar_merger.py',
        name='multi_lidar_merger',
        output='screen',
        parameters=[scan_merger_params],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_params],
    )

    return LaunchDescription([
        merger_node,
        slam_node,
    ])
