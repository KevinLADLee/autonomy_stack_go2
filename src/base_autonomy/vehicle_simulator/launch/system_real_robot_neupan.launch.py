import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  start_real_robot = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('vehicle_simulator'),
        'launch',
        'system_real_robot.launch.py',
      ])
    ),
    launch_arguments={
      'publish2dMap': 'false',
    }.items()
  )

  start_neupan = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('neupan_ros2'), 'launch', 'go2edu.launch.py')
    ),
    launch_arguments={
      'use_rviz': 'false',
    }.items()
  )

  ld = LaunchDescription()
  ld.add_action(start_real_robot)
  ld.add_action(start_neupan)
  return ld
