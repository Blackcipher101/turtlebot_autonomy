import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Try our custom config, fall back to nav2_bringup default
    custom_rviz = '/ros2_ws/configs/rviz/autonomy.rviz'
    default_rviz = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz'
    )
    rviz_config = custom_rviz if os.path.exists(custom_rviz) else default_rviz

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
