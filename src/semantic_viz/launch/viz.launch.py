from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='semantic_viz',
            executable='marker_publisher',
            name='semantic_viz_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
