# USAGE: ros2 launch go2_sport_api go2_sport_api.launch.py network_interface:=eth0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument for network interface
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enp3s0',
        description='Network interface name for unitree ChannelFactory (e.g., enp3s0, wlan0)'
    )
    
    # Create vel_ctrl node with network interface argument
    vel_ctrl_node = Node(
        package='go2_sport_api',
        executable='vel_ctrl',
        name='vel_cmd_repub',
        output='screen',
        arguments=[LaunchConfiguration('network_interface')],
        # prefix='gdb -ex run --args'
    )
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(network_interface_arg)
    ld.add_action(vel_ctrl_node)

    return ld

