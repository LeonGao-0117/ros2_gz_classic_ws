import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Set package name and path
    pkg_name = 'M3Pro_robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Get the complete path of URDF and RViz configuration files
    urdf_file = os.path.join(pkg_share, 'urdf', 'M3Pro_robot_main.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'M3Pro.rviz')

    # 2. Use xacro to parse the file and get the robot_description string
    # Even pure URDF, processing via xacro ensures it's correctly passed as a string, avoiding YAML errors
    robot_desc = xacro.process_file(urdf_file).toxml()
    # Strip comments to prevent parameter parser errors in ROS2 when launching gazebo/ros2_control
    clean_desc = re.sub(r'<!--(.*?)-->', '', robot_desc, flags=re.DOTALL)
    robot_description_config = clean_desc.replace('\n', '')

    # 3. Node: robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )

    # 4. Node: joint_state_publisher_gui
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 5. Node: RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Assemble
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])