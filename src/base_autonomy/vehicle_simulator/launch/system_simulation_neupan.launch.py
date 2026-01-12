import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configuration variables
    world_name = LaunchConfiguration('world_name')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    vehicleX = LaunchConfiguration('vehicleX')
    vehicleY = LaunchConfiguration('vehicleY')
    terrainZ = LaunchConfiguration('terrainZ')
    vehicleYaw = LaunchConfiguration('vehicleYaw')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')

    # Declare launch arguments (matching system_simulation.launch defaults)
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='unity',
        description='World name for visualization'
    )
    declare_vehicleHeight = DeclareLaunchArgument(
        'vehicleHeight',
        default_value='0.366',
        description='Vehicle height'
    )
    declare_sensorOffsetX = DeclareLaunchArgument(
        'sensorOffsetX',
        default_value='0.0',
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
    declare_terrainZ = DeclareLaunchArgument(
        'terrainZ',
        default_value='0.0',
        description='Terrain Z position'
    )
    declare_vehicleYaw = DeclareLaunchArgument(
        'vehicleYaw',
        default_value='0.0',
        description='Initial vehicle yaw angle'
    )
    declare_checkTerrainConn = DeclareLaunchArgument(
        'checkTerrainConn',
        default_value='true',
        description='Check terrain connectivity'
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

    # Transform publishers will be created in a function to handle negative values
    # This is done using OpaqueFunction to evaluate LaunchConfiguration at runtime

    # Include terrain_analysis launch file
    start_terrain_analysis = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
        ),
        launch_arguments={
            'vehicleHeight': vehicleHeight,
        }.items()
    )

    # Include terrain_analysis_ext launch file
    start_terrain_analysis_ext = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
        ),
        launch_arguments={
            'checkTerrainConn': checkTerrainConn,
            'vehicleHeight': vehicleHeight,
        }.items()
    )

    # Include vehicle_simulator launch file
    start_vehicle_simulator = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle_simulator.launch')
        ),
        launch_arguments={
            'vehicleHeight': vehicleHeight,
            'sensorOffsetX': sensorOffsetX,
            'sensorOffsetY': sensorOffsetY,
            'vehicleX': vehicleX,
            'vehicleY': vehicleY,
            'terrainZ': terrainZ,
            'vehicleYaw': vehicleYaw,
        }.items()
    )

    # Include sensor_scan_generation launch file
    start_sensor_scan_generation = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
        )
    )

    # Include visualization_tools launch file
    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
        ),
        launch_arguments={
            'world_name': world_name,
        }.items()
    )

    # ros_tcp_endpoint node
    ros_tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': '0.0.0.0',
            'ROS_TCP_PORT': 10000,
        }]
    )

    # sim_image_repub node
    sim_image_repub_node = Node(
        package='vehicle_simulator',
        executable='sim_image_repub',
        name='sim_image_repub',
        output='screen',
        parameters=[{
            'camera_in_topic': '/camera/image/compressed',
            'camera_raw_out_topic': '/camera/image/raw',
            'sem_in_topic': '/camera/semantic_image/compressed',
            'sem_raw_out_topic': '/camera/semantic_image/raw',
            'depth_in_topic': '/camera/depth/compressed',
            'depth_raw_out_topic': '/camera/depth/raw',
        }]
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

    # Build launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_name)
    ld.add_action(declare_vehicleHeight)
    ld.add_action(declare_sensorOffsetX)
    ld.add_action(declare_sensorOffsetY)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_vehicleX)
    ld.add_action(declare_vehicleY)
    ld.add_action(declare_terrainZ)
    ld.add_action(declare_vehicleYaw)
    ld.add_action(declare_checkTerrainConn)

    # Add launch descriptions and nodes
    ld.add_action(start_neupan)
    ld.add_action(OpaqueFunction(function=create_transform_publishers))
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_terrain_analysis_ext)
    ld.add_action(start_vehicle_simulator)
    ld.add_action(start_sensor_scan_generation)
    ld.add_action(start_visualization_tools)
    ld.add_action(ros_tcp_endpoint_node)
    ld.add_action(sim_image_repub_node)
    ld.add_action(rviz_node)

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

    return [vehicle_trans_publisher, sensor_trans_publisher]
