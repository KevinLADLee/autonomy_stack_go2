from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tare_tsp_interfaces',
            executable='tsp_solver_service.py',
            name='tsp_solver_service',
            output='screen'
        ),
        Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen'
        )
    ])

