import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. slam_toolbox — online async mapping
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('slam_toolbox').find('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': '/ros2_ws/configs/slam_toolbox_params.yaml',
        }.items(),
    )

    # 2. Nav2 bringup — for costmaps + local planner (no global planner needed during explore)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2'),
                'launch', 'navigation2.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': '/ros2_ws/configs/nav2_params.yaml',
        }.items(),
    )

    # 3. explore_lite — frontier-based autonomous exploration
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('explore_lite').find('explore_lite'),
                'launch', 'explore.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox_launch,
        nav2_launch,
        explore_launch,
    ])
