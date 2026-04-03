from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("route_network_navigator")
    config_path = pkg_share + "/config/route_network_navigator.yaml"

    return LaunchDescription(
        [
            Node(
                package="route_network_navigator",
                executable="route_network_navigator_node",
                name="route_network_navigator",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
