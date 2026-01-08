import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

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

  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    ),
    launch_arguments={
      'vehicleHeight': vehicleHeight,
    }.items()
  )

  # Odin1 driver
  start_odin_ros_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('odin_ros_driver'), 'launch', 'odin1_ros2_without_rviz.launch.py')
    )
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rvizGA',
    output='screen',
    arguments=['-d', os.path.join(
      get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')],
    prefix='nice'
  )

  # Static transform publishers
  loam_interface_trans_pub_map = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='loamInterfaceTransPubMap',
    arguments=['0', '0', '0', '0', '0', '0', '/map', '/odom']
  )

  loam_interface_trans_pub_vehicle = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='loamInterfaceTransPubVehicle',
    arguments=['0', '0', '0', '0', '0', '0', '/odin1_base_link', '/sensor']
  )

  # Go2 robot description and visualization
  go2_description_pkg = get_package_share_directory('go2_description')
  go2_xacro_path = os.path.join(go2_description_pkg, 'xacro', 'robot.xacro')
  robot_description_content = Command(['xacro ', go2_xacro_path])
  robot_description = {'robot_description': robot_description_content}
  
  # Go2 joint state publisher (converts /lowstate to /joint_states)
  go2_joint_state_publisher = Node(
    package='go2_sport_api',
    executable='go2_joint_state_publisher',
    name='go2_joint_state_publisher',
    output='screen'
  )
  
  # Robot state publisher for Go2
  robot_description_with_freq = robot_description.copy()
  robot_description_with_freq['publish_frequency'] = 50.0
  go2_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='go2_robot_state_publisher',
    output='screen',
    parameters=[robot_description_with_freq]
  )
  
  # Static transform: vehicle -> base (sensor -> vehicle is published by local_planner.launch)
  vehicle_to_go2_base = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='vehicle_to_go2_base',
    arguments=['0', '0', '0', '0', '0', '0', 'vehicle', 'base']
  )

  # Foxglove bridge with topic filtering to reduce data transfer
  # Only forward essential topics for visualization and control
  foxglove_bridge_node = Node(
    package='foxglove_bridge',
    executable='foxglove_bridge',
    name='foxglove_bridge',
    parameters=[{
      # Topic whitelist - only forward these topics
      'topic_whitelist': [
        # State and localization
        '/state_estimation',
        '/odom',
        # Point clouds (essential for visualization)
        '/registered_scan',
        '/terrain_map',
        # Planning and navigation
        '/path',
        '/way_point',
        '/goal_pose',
        '/free_paths',
        # TF transforms
        '/tf',
        '/tf_static',
        # Robot state
        '/joint_states',
        '/robot_description',
        # Optional: uncomment if needed
        # '/added_obstacles',
        '/terrain_map_ext',
        '/terrain_map',
        '/odin1/path',
        '/odin1/odometry_highfreq',
        '/odin1/image/compressed',
        '/odin1/cloud_slam'
      ],
      'capabilities': ['topics', 'services', 'parameters'],
      'port': 8765,
      'use_compression': True,  # Enable compression to reduce bandwidth
    }]
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
  ld.add_action(start_odin_ros_driver)
  ld.add_action(rviz_node)
  ld.add_action(loam_interface_trans_pub_map)
  ld.add_action(loam_interface_trans_pub_vehicle)
  ld.add_action(go2_joint_state_publisher)
  ld.add_action(go2_robot_state_publisher)
  ld.add_action(vehicle_to_go2_base)
  ld.add_action(foxglove_bridge_node)
  return ld
