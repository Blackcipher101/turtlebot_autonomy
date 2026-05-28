from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='semantic_graph',
            executable='graph_server',
            name='semantic_graph_server',
            parameters=[{
                'use_sim_time': True,
                'graph_save_path': '/ros2_ws/maps/semantic_graph.json',
            }],
            output='screen',
        ),
    ])
