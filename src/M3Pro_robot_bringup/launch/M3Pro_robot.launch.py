import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, SetEnvironmentVariable,
    DeclareLaunchArgument, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)

    pkg_description = get_package_share_directory('M3Pro_robot_description')
    pkg_bringup = get_package_share_directory('M3Pro_robot_bringup')

    main_xacro_path = os.path.join(pkg_description, 'urdf', 'M3Pro_robot_main.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'M3Pro.rviz')
    install_share_path = os.path.dirname(pkg_description)

    gz_resource_paths = [install_share_path]

    if world == 'hospital':
        try:
            pkg_hospital = get_package_share_directory('aws_robomaker_hospital_world')
            world_path = os.path.join(pkg_hospital, 'worlds', 'hospital.sdf')
            gz_resource_paths.append(os.path.join(pkg_hospital, 'models'))
            gz_resource_paths.append(os.path.join(pkg_hospital, 'fuel_models'))
        except Exception:
            print('[WARN] aws_robomaker_hospital_world not found, falling back to default world')
            world_path = os.path.join(pkg_description, 'worlds', 'M3pro_world.sdf')
    elif world == 'small_house':
        try:
            pkg_small_house = get_package_share_directory('aws_robomaker_small_house_world')
            world_path = os.path.join(pkg_small_house, 'worlds', 'small_house.sdf')
            gz_resource_paths.append(os.path.join(pkg_small_house, 'models'))
            gz_resource_paths.append(pkg_small_house)
        except Exception:
            print('[WARN] aws_robomaker_small_house_world not found, falling back to default world')
            world_path = os.path.join(pkg_description, 'worlds', 'M3pro_world.sdf')
    else:
        world_path = os.path.join(pkg_description, 'worlds', 'M3pro_world.sdf')

    gz_resource_path_str = os.pathsep.join(gz_resource_paths)

    # 3. Parse Xacro
    robot_description_value = ParameterValue(
        Command(['xacro ', main_xacro_path]),
        value_type=str
    )

    # 4. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_value,
            'use_sim_time': True,
        }]
    )

    # 5. Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': [f'-r {world_path}']}.items()
    )

    # 6. Spawn robot (per-world position — must avoid furniture collisions)
    if world == 'small_house':
        spawn_xyz = ['2.0', '1.0', '0.15']
    elif world == 'hospital':
        spawn_xyz = ['0.0', '3.0', '0.15']
    else:
        spawn_xyz = ['0.0', '0.0', '0.15']

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'M3Pro',
            '-topic', 'robot_description',
            '-x', spawn_xyz[0], '-y', spawn_xyz[1], '-z', spawn_xyz[2]
        ],
        output='screen',
    )

    # 7. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan_back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # 8. Delay loading controllers
    def create_controller_spawner(name, delay):
        return TimerAction(
            period=delay,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'],
                    output='screen',
                )
            ]
        )

    is_heavy_world = world in ('hospital', 'small_house')
    base_delay = 20.0 if is_heavy_world else 4.0
    step = 5.0 if is_heavy_world else 2.0

    load_joint_state = create_controller_spawner('joint_state_broadcaster', base_delay)
    load_arm_controller = create_controller_spawner('arm_controller', base_delay + step)
    load_gripper_controller = create_controller_spawner('gripper_controller', base_delay + step * 2)
    # TODO: Revert to 'mecanum_controller' when switching back to mecanum drive
    load_diff_drive_controller = create_controller_spawner('diff_drive_controller', base_delay + step * 3)

    # ========================== Core Modifications ==========================
    # 9. Topic relay: /cmd_vel -> diff_drive_controller's unstamped input
    #    TODO: Revert output_topic to '/mecanum_controller/reference_unstamped' for mecanum
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/diff_drive_controller/cmd_vel_unstamped',
            'type': 'geometry_msgs/msg/Twist',
            'qos_overrides./cmd_vel.subscription.reliability': 'best_effort',
            'qos_overrides./cmd_vel.subscription.durability': 'volatile'
        }]
    )
    # ===============================================================

    # 10. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path_str),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib'),

        node_robot_state_publisher,
        bridge,
        gazebo,
        spawn_entity,
        load_joint_state,
        load_arm_controller,
        load_gripper_controller,
        load_diff_drive_controller,
        cmd_vel_relay,
        rviz,
    ]


def generate_launch_description():
    # 0. Clean up resources
    print("Cleaning up resources...")
    os.system('pkill -9 gzserver; pkill -9 gzclient; pkill -9 ruby; pkill -9 robot_state_publisher; pkill -9 rviz2 > /dev/null 2>&1')
    time.sleep(2.0)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='default',
            description='World to load: "default" for M3pro_world, "hospital" for AWS hospital, "small_house" for AWS small house'
        ),
        OpaqueFunction(function=launch_setup),
    ])
