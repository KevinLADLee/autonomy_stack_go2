# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _declare_common_args(package_dir):
    default_output_root = os.path.expanduser('~/.ros/odin_ros_driver')
    return [
        DeclareLaunchArgument('config_file', default_value=os.path.join(package_dir, 'config', 'control_command.yaml'), description='Path to the control config YAML file'),
        DeclareLaunchArgument('output_root', default_value=default_output_root, description='Root directory for runtime outputs'),
        DeclareLaunchArgument('calib_file', default_value=os.path.join(default_output_root, 'calib', 'calib.yaml'), description='Path to calib.yaml output'),
        DeclareLaunchArgument('command_file', default_value='/tmp/odin_command.txt', description='Path to the command file'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'), description='Path to RViz2 config file'),
        DeclareLaunchArgument('topics.imu', default_value='odin1/imu', description='IMU topic'),
        DeclareLaunchArgument('topics.image', default_value='odin1/image', description='RGB image topic'),
        DeclareLaunchArgument('topics.cloud_raw', default_value='odin1/cloud_raw', description='Raw cloud topic'),
        DeclareLaunchArgument('topics.cloud_slam', default_value='/registered_scan', description='SLAM cloud topic'),
        DeclareLaunchArgument('topics.odometry', default_value='odin1/odometry', description='Odometry topic'),
        DeclareLaunchArgument('topics.odometry_highfreq', default_value='/state_estimation', description='High-frequency odometry topic'),
        DeclareLaunchArgument('topics.path', default_value='odin1/path', description='Path topic'),
        DeclareLaunchArgument('topics.camera_pose_visual', default_value='odin1/camera_pose_visual', description='Camera pose visualization topic'),
        DeclareLaunchArgument('topics.cloud_render', default_value='odin1/cloud_render', description='Rendered cloud topic'),
        DeclareLaunchArgument('topics.image_compressed', default_value='odin1/image/compressed', description='Compressed image topic'),
        DeclareLaunchArgument('topics.image_undistorted', default_value='odin1/image/undistorted', description='Undistorted image topic'),
        DeclareLaunchArgument('topics.image_intensity_gray', default_value='odin1/image/intensity_gray', description='Intensity image topic'),
        DeclareLaunchArgument('topics.color_raw', default_value='/odin1/image', description='Depth projector color image topic'),
        DeclareLaunchArgument('topics.color_compressed', default_value='/odin1/image/compressed', description='Depth projector compressed image topic'),
        DeclareLaunchArgument('topics.depth_image', default_value='/odin1/depth_img_competetion', description='Depth image topic'),
        DeclareLaunchArgument('topics.depth_cloud', default_value='/odin1/depth_img_competetion_cloud', description='Depth point cloud topic'),
        DeclareLaunchArgument('topics.reprojected_image', default_value='/odin1/reprojected_image', description='Reprojected image topic'),
    ]


def _common_parameters():
    return {
        'config_file': LaunchConfiguration('config_file'),
        'output_root': LaunchConfiguration('output_root'),
        'calib_file': LaunchConfiguration('calib_file'),
        'command_file': LaunchConfiguration('command_file'),
        'topics.imu': LaunchConfiguration('topics.imu'),
        'topics.image': LaunchConfiguration('topics.image'),
        'topics.cloud_raw': LaunchConfiguration('topics.cloud_raw'),
        'topics.cloud_slam': LaunchConfiguration('topics.cloud_slam'),
        'topics.odometry': LaunchConfiguration('topics.odometry'),
        'topics.odometry_highfreq': LaunchConfiguration('topics.odometry_highfreq'),
        'topics.path': LaunchConfiguration('topics.path'),
        'topics.camera_pose_visual': LaunchConfiguration('topics.camera_pose_visual'),
        'topics.cloud_render': LaunchConfiguration('topics.cloud_render'),
        'topics.image_compressed': LaunchConfiguration('topics.image_compressed'),
        'topics.image_undistorted': LaunchConfiguration('topics.image_undistorted'),
        'topics.image_intensity_gray': LaunchConfiguration('topics.image_intensity_gray'),
    }


def _depth_parameters():
    return {
        'config_file': LaunchConfiguration('config_file'),
        'output_root': LaunchConfiguration('output_root'),
        'calib_file': LaunchConfiguration('calib_file'),
        'topics.cloud_raw': LaunchConfiguration('topics.cloud_raw'),
        'topics.color_raw': LaunchConfiguration('topics.color_raw'),
        'topics.color_compressed': LaunchConfiguration('topics.color_compressed'),
        'topics.depth_image': LaunchConfiguration('topics.depth_image'),
        'topics.depth_cloud': LaunchConfiguration('topics.depth_cloud'),
    }


def _reprojection_parameters():
    return {
        'config_file': LaunchConfiguration('config_file'),
        'output_root': LaunchConfiguration('output_root'),
        'calib_file': LaunchConfiguration('calib_file'),
        'topics.cloud_slam': LaunchConfiguration('topics.cloud_slam'),
        'topics.odometry': LaunchConfiguration('topics.odometry'),
        'topics.reprojected_image': LaunchConfiguration('topics.reprojected_image'),
    }


def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[_common_parameters()],
    )

    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[_depth_parameters()],
    )

    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[_reprojection_parameters()],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    ld = LaunchDescription()
    for action in _declare_common_args(package_dir):
        ld.add_action(action)
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(rviz_node)
    return ld
