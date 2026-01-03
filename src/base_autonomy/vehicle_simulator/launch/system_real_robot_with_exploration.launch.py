import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
  exploration_planner_config = LaunchConfiguration('exploration_planner_config')
  world_name = LaunchConfiguration('world_name')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')

  declare_exploration_planner_config = DeclareLaunchArgument('exploration_planner_config', default_value='indoor_small', description='')
  declare_world_name = DeclareLaunchArgument('world_name', default_value='real_world', description='')
  declare_sensorOffsetX = DeclareLaunchArgument('sensorOffsetX', default_value='0.05', description='')
  declare_sensorOffsetY = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.25', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  
  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'realRobot': 'true',
      'sensorOffsetX': sensorOffsetX,
      'sensorOffsetY': sensorOffsetY,
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
    }.items()
  )

  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    )
  )

  start_terrain_analysis_ext = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
    ),
    launch_arguments={
      'checkTerrainConn': checkTerrainConn,
    }.items()
  )

  start_sensor_scan_generation = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
    )
  )

  # start_visualization_tools = IncludeLaunchDescription(
  #   FrontendLaunchDescriptionSource(os.path.join(
  #     get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
  #   ),
  #   launch_arguments={
  #     'world_name': world_name,
  #   }.items()
  # )

  start_tare_planner = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [get_package_share_directory('tare_planner'), '/explore_world.launch']),
    launch_arguments={
      'scenario': exploration_planner_config,
    }.items()
  )

  # TSP Solver Service Node
  tsp_solver_service = Node(
    package='tare_tsp_interfaces',
    executable='tsp_solver_service.py',
    name='tsp_solver_service',
    output='screen'
  )

  # Static transform publishers
  odin_interface_trans_pub_map = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odinInterfaceTransPubMap',
    arguments=['0', '0', '0', '0', '0', '0', '/map', '/odom']
  )

  odin_interface_trans_pub_vehicle = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odinInterfaceTransPubVehicle',
    arguments=['0', '0', '0', '0', '0', '0', '/odin1_base_link', '/sensor']
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_exploration_planner_config)
  ld.add_action(declare_world_name)
  ld.add_action(declare_sensorOffsetX)
  ld.add_action(declare_sensorOffsetY)
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_checkTerrainConn)

  ld.add_action(start_local_planner)
  ld.add_action(start_terrain_analysis)
  ld.add_action(start_terrain_analysis_ext)
  ld.add_action(start_sensor_scan_generation) 
  # ld.add_action(start_visualization_tools)
  # Start TSP solver service before tare_planner
  ld.add_action(tsp_solver_service)
  ld.add_action(start_tare_planner)
  ld.add_action(odin_interface_trans_pub_map)
  ld.add_action(odin_interface_trans_pub_vehicle)

  return ld