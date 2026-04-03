import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  check_terrain_conn = LaunchConfiguration('checkTerrainConn')
  vehicle_height = LaunchConfiguration('vehicleHeight')

  declare_check_terrain_conn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  declare_vehicle_height = DeclareLaunchArgument('vehicleHeight', default_value='0.40', description='')

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

  start_terrain_analysis_ext = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
    ),
    launch_arguments={
      'checkTerrainConn': check_terrain_conn,
      'vehicleHeight': vehicle_height,
    }.items()
  )

  start_far_planner = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('far_planner'), 'launch', 'far_planner.launch')
    )
  )

  ld = LaunchDescription()
  ld.add_action(declare_check_terrain_conn)
  ld.add_action(declare_vehicle_height)
  ld.add_action(start_real_robot)
  ld.add_action(start_terrain_analysis_ext)
  ld.add_action(start_far_planner)
  return ld
