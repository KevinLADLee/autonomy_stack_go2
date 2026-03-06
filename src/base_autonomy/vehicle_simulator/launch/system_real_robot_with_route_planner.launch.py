import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  world_name = LaunchConfiguration('world_name')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  vehicleHeight = LaunchConfiguration('vehicleHeight')

  declare_world_name = DeclareLaunchArgument('world_name', default_value='real_world', description='')
  declare_sensorOffsetX = DeclareLaunchArgument('sensorOffsetX', default_value='0.3', description='')
  declare_sensorOffsetY = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  declare_vehicleHeight = DeclareLaunchArgument('vehicleHeight', default_value='0.366', description='')

  # <include file="$(find-pkg-share local_planner)/launch/local_planner.launch" >
  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'sensorOffsetX': sensorOffsetX,
      'sensorOffsetY': sensorOffsetY,
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
    }.items()
  )

  # <include file="$(find-pkg-share terrain_analysis)/launch/terrain_analysis.launch" />
  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    ),
    launch_arguments={
      'vehicleHeight': vehicleHeight,
    }.items()
  )

  # <include file="$(find-pkg-share terrain_analysis_ext)/launch/terrain_analysis_ext.launch" >
  start_terrain_analysis_ext = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
    ),
    launch_arguments={
      'checkTerrainConn': checkTerrainConn,
      'vehicleHeight': vehicleHeight,
    }.items()
  )

  # <include file="$(find-pkg-share far_planner)/launch/far_planner.launch" />
  start_far_planner = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('far_planner'), 'launch', 'far_planner.launch')
    )
  )

  # Odin1 driver
  start_odin_ros_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('odin_ros_driver'), 'launch', 'odin1_ros2_without_rviz.launch.py')
    )
  )

  # Static transform publishers
  # <node pkg="tf2_ros" exec="static_transform_publisher" name="loamInterfaceTransPubMap" args="0 0 0 0 0 0 /map /odom"/>
  loam_interface_trans_pub_map = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='loamInterfaceTransPubMap',
    arguments=['0', '0', '0', '0', '0', '0', '/map', '/odom']
  )

  # <node pkg="tf2_ros" exec="static_transform_publisher" name="loamInterfaceTransPubVehicle" args="0 0 0 0 0 0 /odin1_base_link /sensor"/>
  loam_interface_trans_pub_vehicle = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='loamInterfaceTransPubVehicle',
    arguments=['0', '0', '0', '0', '0', '0', '/odin1_base_link', '/sensor']
  )

  ld = LaunchDescription()

  # Add the launch arguments
  ld.add_action(declare_world_name)
  ld.add_action(declare_sensorOffsetX)
  ld.add_action(declare_sensorOffsetY)
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_checkTerrainConn)
  ld.add_action(declare_vehicleHeight)

  # Add the actions
  ld.add_action(start_local_planner)
  ld.add_action(start_terrain_analysis)
  ld.add_action(start_terrain_analysis_ext)
  ld.add_action(start_far_planner)
  ld.add_action(start_odin_ros_driver)
  ld.add_action(loam_interface_trans_pub_map)
  ld.add_action(loam_interface_trans_pub_vehicle)

  return ld
