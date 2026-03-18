import os
import re
import xacro
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    DeclareLaunchArgument, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def _prepend_env(name, value):
    """Prepend a value to an environment variable (or create it)."""
    existing = os.environ.get(name, '')
    if existing:
        os.environ[name] = value + os.pathsep + existing
    else:
        os.environ[name] = value


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)

    pkg_description = get_package_share_directory('M3Pro_robot_description')
    pkg_bringup = get_package_share_directory('M3Pro_robot_bringup')

    main_xacro_path = os.path.join(pkg_description, 'urdf', 'M3Pro_robot_main.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'M3Pro.rviz')
    install_share_path = os.path.dirname(pkg_description)

    gazebo_model_paths = [install_share_path]

    if world == 'hospital':
        try:
            pkg_hospital = get_package_share_directory('aws_robomaker_hospital_world')
            world_path = os.path.join(pkg_hospital, 'worlds', 'hospital.world')
            gazebo_model_paths.append(os.path.join(pkg_hospital, 'models'))
            gazebo_model_paths.append(os.path.join(pkg_hospital, 'fuel_models'))
        except Exception:
            print('[WARN] aws_robomaker_hospital_world not found, falling back to default world')
            world_path = os.path.join(pkg_description, 'worlds', 'M3pro_world.world')
    elif world == 'small_house':
        try:
            pkg_small_house = get_package_share_directory('aws_robomaker_small_house_world')
            world_path = os.path.join(pkg_small_house, 'worlds', 'small_house.world')
            gazebo_model_paths.append(os.path.join(pkg_small_house, 'models'))
            gazebo_model_paths.append(pkg_small_house)
        except Exception:
            print('[WARN] aws_robomaker_small_house_world not found, falling back to default world')
            world_path = os.path.join(pkg_description, 'worlds', 'M3pro_world.world')
    else:
        world_path = os.path.join(pkg_description, 'worlds', 'M3pro_world.world')

    gazebo_model_path_str = os.pathsep.join(gazebo_model_paths)

    # Set GAZEBO env vars directly in os.environ so that gzserver.launch.py
    # (which reads os.environ at import time) picks them up correctly.
    # SetEnvironmentVariable is too late — gzserver.launch.py reads environ
    # during generate_launch_description(), not at process spawn time.
    _prepend_env('GAZEBO_MODEL_PATH', gazebo_model_path_str)
    _prepend_env('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib')

    # 3. Parse Xacro and strip comments and newlines
    doc = xacro.process_file(main_xacro_path)
    robot_desc = doc.toxml()
    # Strip comments to prevent parameter parser errors in gazebo_ros2_control
    clean_desc = re.sub(r'<!--(.*?)-->', '', robot_desc, flags=re.DOTALL)
    clean_desc = clean_desc.replace('\n', '')
    
    robot_description_value = ParameterValue(
        clean_desc,
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

    # 5. Start Gazebo Classic (gzserver + gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # 6. Spawn robot (per-world position — must avoid furniture collisions)
    if world == 'small_house':
        spawn_xyz = ['2.0', '1.0', '0.15']
    elif world == 'hospital':
        spawn_xyz = ['0.0', '3.0', '0.15']
    else:
        spawn_xyz = ['0.0', '0.0', '0.15']

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'M3Pro',
            '-topic', 'robot_description',
            '-x', spawn_xyz[0], '-y', spawn_xyz[1], '-z', spawn_xyz[2]
        ],
        output='screen',
    )

    # 7. Gazebo Classic plugins publish directly to ROS 2 topics — no bridge needed

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
        node_robot_state_publisher,
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
