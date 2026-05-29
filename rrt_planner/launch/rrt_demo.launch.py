#!/usr/bin/env python3
"""Launch: map_server (lifecycle) + rrt_planner_node + RViz2.

Map source: TurtleBot3 house world explored with SLAM Toolbox,
saved as /workspace/ros_ws/maps/explored_map.yaml + explored_map.pgm

Usage:
  ros2 launch rrt_planner rrt_demo.launch.py
  ros2 launch rrt_planner rrt_demo.launch.py map:=/path/to/other_map.yaml
  ros2 launch rrt_planner rrt_demo.launch.py use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rrt_planner')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/workspace/ros_ws/maps/explored_map.yaml',
        description='Path to map YAML file (nav2_map_server format)',
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use Gazebo sim clock',
    )
    rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'rrt_params.yaml'),
        description='RRT planner parameter file',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map'),
        }],
    )

    # nav2_lifecycle_manager transitions map_server: unconfigured→active
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    rrt_node = Node(
        package='rrt_planner',
        executable='rrt_planner_node',
        name='rrt_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'rrt_rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        map_arg, sim_time_arg, rviz_arg, params_arg,
        map_server,
        lifecycle_mgr,
        rrt_node,
        rviz_node,
    ])
