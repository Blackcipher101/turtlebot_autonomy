from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='semantic_navigation',
            executable='query_node',
            name='semantic_query_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
