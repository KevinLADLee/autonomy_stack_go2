import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    # Launch configuration variables
    world_name = LaunchConfiguration('world_name')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    vehicleX = LaunchConfiguration('vehicleX')
    vehicleY = LaunchConfiguration('vehicleY')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    vehicleHeight = LaunchConfiguration('vehicleHeight')

    # Declare launch arguments (matching system_real_robot.launch.py defaults)
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='real_world',
        description='World name for visualization'
    )
    declare_sensorOffsetX = DeclareLaunchArgument(
        'sensorOffsetX',
        default_value='0.3',
        description='Sensor offset X'
    )
    declare_sensorOffsetY = DeclareLaunchArgument(
        'sensorOffsetY',
        default_value='0.0',
        description='Sensor offset Y'
    )
    declare_cameraOffsetZ = DeclareLaunchArgument(
        'cameraOffsetZ',
        default_value='0.0',
        description='Camera offset Z'
    )
    declare_vehicleX = DeclareLaunchArgument(
        'vehicleX',
        default_value='0.0',
        description='Initial vehicle X position'
    )
    declare_vehicleY = DeclareLaunchArgument(
        'vehicleY',
        default_value='0.0',
        description='Initial vehicle Y position'
    )
    declare_checkTerrainConn = DeclareLaunchArgument(
        'checkTerrainConn',
        default_value='true',
        description='Check terrain connectivity'
    )
    declare_vehicleHeight = DeclareLaunchArgument(
        'vehicleHeight',
        default_value='0.40',
        description='Vehicle height'
    )

    # Include NeuPAN launch file (go2edu.launch.py)
    # Disable rviz in go2edu.launch.py since we have our own rviz
    start_neupan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('neupan_ros2'), 'launch', 'go2edu.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )

    # Include terrain_analysis launch file
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

    # rviz2 node
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
    # map -> odom
    loam_interface_trans_pub_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='loamInterfaceTransPubMap',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # odin1_base_link -> sensor
    loam_interface_trans_pub_vehicle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='loamInterfaceTransPubVehicle',
        arguments=['0', '0', '0', '0', '0', '0', 'odin1_base_link', 'sensor']
    )

    # Transform publishers will be created in a function to handle negative values
    # This includes: sensor->vehicle, sensor->camera, sensor->base

    # Go2 Sport API
    start_go2_sport_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('go2_sport_api'), 'launch', 'go2_sport_api.launch.py')
        )
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

    # Foxglove bridge with topic filtering to reduce data transfer
    # Only forward essential topics for visualization and control
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            # Topic whitelist - only forward these topics
            'topic_whitelist': [
                '/added_obstacles',
                '/api/sport/request',
                '/check_obstacle',
                '/cmd_vel',
                '/dune_point_markers',  # NeuPAN DUNE visualization
                '/free_paths',
                '/goal_pose',
                '/joint_states',
                '/joy',
                '/lowstate',
                '/map_clearing',
                '/navigation_boundary',
                '/neupan_cmd_vel',  # NeuPAN velocity commands
                '/neupan_initial_path',  # NeuPAN initial path visualization
                '/neupan_plan',  # NeuPAN optimized trajectory
                '/neupan_ref_state',  # NeuPAN reference state trajectory
                '/nrmp_point_markers',  # NeuPAN NRMP visualization
                '/odin/cloud_raw',
                '/odin1/camera_pose_visual',
                '/odin1/cloud_render',
                '/odin1/image',
                '/odin1/image/compressed',
                '/odin1/image/intensity_gray',
                '/odin1/image/undistorted',
                '/odin1/imu',
                '/odin1/odometry_highfreq',
                '/odin1/path',
                '/overall_map',
                '/parameter_events',
                '/path',
                '/registered_scan',
                '/robot_description',
                '/robot_marker',  # NeuPAN robot footprint visualization
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
            'use_compression': True,  # Enable compression to reduce bandwidth
        }]
    )

    # Build launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_name)
    ld.add_action(declare_sensorOffsetX)
    ld.add_action(declare_sensorOffsetY)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_vehicleX)
    ld.add_action(declare_vehicleY)
    ld.add_action(declare_checkTerrainConn)
    ld.add_action(declare_vehicleHeight)

    # Add launch descriptions and nodes
    ld.add_action(start_neupan)
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_odin_ros_driver)
    ld.add_action(start_go2_sport_api)
    ld.add_action(rviz_node)
    ld.add_action(loam_interface_trans_pub_map)
    ld.add_action(loam_interface_trans_pub_vehicle)
    ld.add_action(OpaqueFunction(function=create_transform_publishers))
    ld.add_action(go2_joint_state_publisher)
    ld.add_action(go2_robot_state_publisher)
    ld.add_action(foxglove_bridge_node)

    return ld


def create_transform_publishers(context, *args, **kwargs):
    """Create transform publisher nodes with negative values from LaunchConfiguration."""
    # Get launch configuration values
    sensor_offset_x = LaunchConfiguration('sensorOffsetX').perform(context)
    sensor_offset_y = LaunchConfiguration('sensorOffsetY').perform(context)
    camera_offset_z = LaunchConfiguration('cameraOffsetZ').perform(context)

    # vehicleTransPublisher: /sensor -> /vehicle
    # XML: args="-$(var sensorOffsetX) -$(var sensorOffsetY) 0 0 0 0 /sensor /vehicle"
    # Format: x y z yaw pitch roll frame_id child_frame_id
    vehicle_trans_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicleTransPublisher',
        arguments=[
            f'-{sensor_offset_x}',
            f'-{sensor_offset_y}',
            '0', '0', '0', '0',
            'sensor', 'vehicle'
        ]
    )

    # sensorTransPublisher: /sensor -> /camera
    # XML: args="0 0 $(var cameraOffsetZ) -1.5707963 0 -1.5707963 /sensor /camera"
    sensor_trans_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensorTransPublisher',
        arguments=[
            '0', '0',
            camera_offset_z,
            '-1.5707963', '0', '-1.5707963',
            'sensor', 'camera'
        ]
    )

    # Static transform: sensor -> base (from system_real_robot.launch.py)
    # This connects the sensor frame to the Go2 base_link frame
    vehicle_to_go2_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicle_to_go2_base',
        arguments=['-0.3', '0', '0', '0', '0', '0', 'sensor', 'base']
    )

    return [vehicle_trans_publisher, sensor_trans_publisher, vehicle_to_go2_base]
