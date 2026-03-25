import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('M3Pro_robot_navigation')

    # SHARED CONFIG: scan_merger.yaml is used by both SLAM and Navigation.
    # Keep sensor fusion behavior consistent between mapping and localization.
    scan_merger_params = os.path.join(pkg_share, 'config', 'scan_merger.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_scan_merger = LaunchConfiguration('use_scan_merger')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for all navigation nodes.',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to map yaml file.',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Absolute path to Nav2 parameters yaml file.',
    )

    declare_use_scan_merger = DeclareLaunchArgument(
        'use_scan_merger',
        default_value='true',
        description='Start shared multi_lidar_merger node to publish /scan_merged for AMCL.',
    )

    merger_node = Node(
        package='M3Pro_robot_navigation',
        executable='multi_lidar_merger.py',
        name='multi_lidar_merger',
        output='screen',
        parameters=[scan_merger_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_scan_merger),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            },
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_use_scan_merger,
        merger_node,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
