from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  publish_2d_map = LaunchConfiguration('publish2dMap')

  declare_publish_2d_map = DeclareLaunchArgument(
    'publish2dMap',
    default_value='true',
    description='Enable 2D occupancy map publisher in real robot stack',
  )

  start_real_robot_common = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('vehicle_simulator'),
        'launch',
        'system_real_robot_common.launch.py',
      ])
    ),
    launch_arguments={
      'publish2dMap': publish_2d_map,
    }.items()
  )

  ld = LaunchDescription()
  ld.add_action(declare_publish_2d_map)
  ld.add_action(start_real_robot_common)
  return ld
