import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  sensor_offset_x = LaunchConfiguration('sensorOffsetX')
  sensor_offset_y = LaunchConfiguration('sensorOffsetY')
  camera_offset_z = LaunchConfiguration('cameraOffsetZ')
  vehicle_x = LaunchConfiguration('vehicleX')
  vehicle_y = LaunchConfiguration('vehicleY')
  vehicle_height = LaunchConfiguration('vehicleHeight')
  publish_global_map = LaunchConfiguration('publishGlobalMap')
  global_map_pcd_file = LaunchConfiguration('globalMapPcdFile')
  global_map_source_topic = LaunchConfiguration('globalMapSourceTopic')
  global_map_frame = LaunchConfiguration('globalMapFrame')
  global_map_topic = LaunchConfiguration('globalMapTopic')
  publish_2d_map = LaunchConfiguration('publish2dMap')
  map_yaml_file = LaunchConfiguration('mapYamlFile')
  map_topic = LaunchConfiguration('mapTopic')
  map_frame = LaunchConfiguration('mapFrame')

  declare_world_name = DeclareLaunchArgument('world_name', default_value='real_world', description='')
  declare_sensor_offset_x = DeclareLaunchArgument('sensorOffsetX', default_value='0.3', description='')
  declare_sensor_offset_y = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
  declare_camera_offset_z = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicle_x = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicle_y = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_check_terrain_conn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  declare_vehicle_height = DeclareLaunchArgument('vehicleHeight', default_value='0.40', description='')
  declare_publish_global_map = DeclareLaunchArgument('publishGlobalMap', default_value='true', description='')
  declare_global_map_pcd_file = DeclareLaunchArgument(
    'globalMapPcdFile',
    default_value='/home/nv/autonomy_stack_go2/src/utilities/odin_maps/siat-f10-0324-map/map_cloud_cleaned_0.05.pcd',
    description='')
  declare_global_map_source_topic = DeclareLaunchArgument(
    'globalMapSourceTopic',
    default_value='/odin1/cloud_slam',
    description='')
  declare_global_map_frame = DeclareLaunchArgument('globalMapFrame', default_value='map', description='')
  declare_global_map_topic = DeclareLaunchArgument('globalMapTopic', default_value='/overall_map', description='')
  declare_map_yaml_file = DeclareLaunchArgument(
    'mapYamlFile',
    default_value='/home/nv/autonomy_stack_go2/src/utilities/odin_maps/siat-f10-0324-map/map.yaml',
    description='')
  declare_map_topic = DeclareLaunchArgument('mapTopic', default_value='/map', description='')
  declare_map_frame = DeclareLaunchArgument('mapFrame', default_value='map', description='')
  declare_publish_2d_map = DeclareLaunchArgument('publish2dMap', default_value='true', description='')

  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'sensorOffsetX': sensor_offset_x,
      'sensorOffsetY': sensor_offset_y,
      'cameraOffsetZ': camera_offset_z,
      'goalX': vehicle_x,
      'goalY': vehicle_y,
    }.items()
  )

  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    ),
    launch_arguments={
      'vehicleHeight': vehicle_height,
    }.items()
  )

  start_odin_ros_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('odin_ros_driver'), 'launch', 'odin1_ros2_without_rviz.launch.py')
    ),
    launch_arguments={
      'topics.cloud_slam': '/odin1/cloud_slam',
      'topics.odometry_highfreq': '/odin1/highodom',
    }.items()
  )

  start_go2_sport_api = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('go2_sport_api'), 'launch', 'go2_sport_api.launch.py')
    )
  )

  static_pcd_map_publisher = Node(
    package='visualization_tools',
    executable='static_pcd_map_publisher',
    name='static_pcd_map_publisher',
    output='screen',
    parameters=[{
      'pcd_file': global_map_pcd_file,
      'source_topic': global_map_source_topic,
      'frame_id': global_map_frame,
      'topic_name': global_map_topic,
      'publish_delay_ms': 1000,
    }],
    condition=IfCondition(publish_global_map)
  )

  static_occupancy_map_publisher = Node(
    package='visualization_tools',
    executable='static_occupancy_map_publisher',
    name='static_occupancy_map_publisher',
    output='screen',
    parameters=[{
      'map_yaml_file': map_yaml_file,
      'topic_name': map_topic,
      'frame_id': map_frame,
      'publish_delay_ms': 1000,
    }],
    condition=IfCondition(publish_2d_map)
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

  odin_adapter = Node(
    package='odin_adapter',
    executable='odin_adapter_node',
    name='odin_adapter',
    output='screen',
    parameters=[{
      'raw_odom_topic': '/odin1/highodom',
      'raw_cloud_topic': '/odin1/cloud_slam',
      'output_odom_topic': '/state_estimation',
      'output_cloud_topic': '/registered_scan',
      'map_frame': 'map',
      'publish_fallback_before_tf': False,
      'z_filter_min': -1.0,
      'z_filter_max': 2.0,
    }]
  )

  loam_interface_trans_pub_vehicle = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='loamInterfaceTransPubVehicle',
    arguments=['0', '0', '0', '0', '0', '0', 'odin1_base_link', 'sensor']
  )

  vehicle_to_go2_base = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='vehicle_to_go2_base',
    arguments=['-0.3', '0', '0', '0', '0', '0', 'sensor', 'base']
  )

  go2_description_pkg = get_package_share_directory('go2_description')
  go2_xacro_path = os.path.join(go2_description_pkg, 'xacro', 'robot.xacro')
  robot_description_content = Command(['xacro ', go2_xacro_path])
  robot_description_with_freq = {
    'robot_description': robot_description_content,
    'publish_frequency': 50.0,
  }

  go2_joint_state_publisher = Node(
    package='go2_sport_api',
    executable='go2_joint_state_publisher',
    name='go2_joint_state_publisher',
    output='screen'
  )

  go2_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='go2_robot_state_publisher',
    output='screen',
    parameters=[robot_description_with_freq]
  )

  foxglove_bridge_node = Node(
    package='foxglove_bridge',
    executable='foxglove_bridge',
    name='foxglove_bridge',
    parameters=[{
      'topic_whitelist': [
        '/added_obstacles',
        '/api/sport/request',
        '/check_obstacle',
        '/cmd_vel',
        '/free_paths',
        '/goal_pose',
        '/joint_states',
        '/joy',
        '/lowstate',
        '/map',
        '/map_clearing',
        '/navigation_boundary',
        '/odin/cloud_raw',
        '/odin1/camera_pose_visual',
        '/odin1/cloud_render',
        '/odin1/image',
        '/odin1/image/compressed',
        '/odin1/image/intensity_gray',
        '/odin1/image/undistorted',
        '/odin1/imu',
        '/odin1/highodom',
        '/odin1/path',
        '/overall_map',
        '/parameter_events',
        '/registered_scan',
        '/path',
        '/robot_description',
        '/rosout',
        '/speed',
        '/state_estimation',
        '/stop',
        '/terrain_map',
        '/tf',
        '/tf_static',
        '/wirelesscontroller',
        '/lf/battery_alarm'
      ],
      'capabilities': ['topics', 'services', 'parameters'],
      'port': 8765,
      'use_compression': True,
    }]
  )

  ld = LaunchDescription()
  ld.add_action(declare_world_name)
  ld.add_action(declare_sensor_offset_x)
  ld.add_action(declare_sensor_offset_y)
  ld.add_action(declare_camera_offset_z)
  ld.add_action(declare_vehicle_x)
  ld.add_action(declare_vehicle_y)
  ld.add_action(declare_check_terrain_conn)
  ld.add_action(declare_vehicle_height)
  ld.add_action(declare_publish_global_map)
  ld.add_action(declare_global_map_pcd_file)
  ld.add_action(declare_global_map_source_topic)
  ld.add_action(declare_global_map_frame)
  ld.add_action(declare_global_map_topic)
  ld.add_action(declare_publish_2d_map)
  ld.add_action(declare_map_yaml_file)
  ld.add_action(declare_map_topic)
  ld.add_action(declare_map_frame)

  ld.add_action(start_local_planner)
  ld.add_action(start_terrain_analysis)
  ld.add_action(start_odin_ros_driver)
  ld.add_action(start_go2_sport_api)
  ld.add_action(static_pcd_map_publisher)
  ld.add_action(static_occupancy_map_publisher)
  ld.add_action(rviz_node)
  ld.add_action(odin_adapter)
  ld.add_action(loam_interface_trans_pub_vehicle)
  ld.add_action(go2_joint_state_publisher)
  ld.add_action(go2_robot_state_publisher)
  ld.add_action(vehicle_to_go2_base)
  ld.add_action(foxglove_bridge_node)
  return ld
