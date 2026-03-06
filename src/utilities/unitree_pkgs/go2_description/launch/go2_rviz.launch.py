#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('go2_description')
    
    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Whether to use joint_state_publisher_gui'
    )
    
    # Robot description using xacro
    xacro_path = os.path.join(package_dir, 'xacro', 'robot.xacro')
    
    # Use xacro command to process the xacro file
    robot_description_content = Command([
        'xacro ', xacro_path
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Joint state publisher (GUI or regular)
    use_gui = LaunchConfiguration('use_gui')
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # RViz2
    rviz_config_path = os.path.join(package_dir, 'launch', 'check_joint.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        respawn=False
    )
    
    return LaunchDescription([
        use_gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])

