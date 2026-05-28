from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='semantic_perception',
            executable='perception_node',
            name='semantic_perception_node',
            parameters=[{
                'use_sim_time': True,
                'min_travel_distance': 1.5,
                'periodic_interval': 30.0,
                'dino_confidence': 0.35,
            }],
            output='screen',
        ),
    ])
