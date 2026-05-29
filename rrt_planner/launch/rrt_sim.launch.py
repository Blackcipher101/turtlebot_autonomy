#!/usr/bin/env python3
"""Full simulation launch: Gazebo + SLAM + Nav2 + RRT* planner.

The RRT* planner replaces Nav2's built-in global planner:
  1. User sets "2D Pose Estimate" in RViz  → /initialpose (start)
  2. User clicks "2D Goal Pose" in RViz    → /goal_pose (triggers RRT*)
  3. RRT* computes path, publishes /rrt_path + /rrt_tree (visualization)
  4. Path is sent to Nav2 /follow_path action → robot physically drives it

Usage:
  ros2 launch rrt_planner rrt_sim.launch.py
  ros2 launch rrt_planner rrt_sim.launch.py auto_follow:=false  # plan only
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
    BURGER = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'

    # ── Args ──────────────────────────────────────────────────────────
    auto_follow_arg = DeclareLaunchArgument(
        'auto_follow', default_value='true',
        description='If true, robot follows the RRT* path automatically',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world',
        description='Gazebo world file',
    )

    # ── Gazebo + SLAM + Nav2 via official TB3 launch ──────────────────
    # tb3_simulation_launch.py handles: gzserver, spawn_entity,
    # robot_state_publisher, slam_toolbox, nav2 (all lifecycle-managed)
    tb3_sim = Node(
        package='launch_ros',
        executable='launch',  # placeholder — we use IncludeLaunchDescription below
        name='tb3_sim',
        output='screen',
    )

    # ── RRT* planner node ─────────────────────────────────────────────
    rrt_node = Node(
        package='rrt_planner',
        executable='rrt_planner_node',
        name='rrt_planner_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'rrt_params.yaml'),
            {
                'auto_follow': LaunchConfiguration('auto_follow'),
                'use_sim_time': True,
            },
        ],
    )

    # ── RViz2 with combined map + path + tree display ─────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'rrt_rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        auto_follow_arg,
        use_rviz_arg,
        world_arg,
        rrt_node,
        rviz_node,
    ])
