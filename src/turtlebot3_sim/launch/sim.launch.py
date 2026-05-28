import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_tb3_gazebo = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    pkg_this       = FindPackageShare('turtlebot3_sim').find('turtlebot3_sim')

    world_file   = os.path.join(pkg_this, 'worlds', 'office_semantic.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless     = LaunchConfiguration('headless', default='true')

    # gzserver only — no gzclient (headless, works inside Docker on macOS)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'extra_gazebo_args': '-s libgazebo_ros_factory.so -s libgazebo_ros_init.so',
        }.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless',     default_value='true'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(pkg_tb3_gazebo, 'models') + ':' +
            '/opt/ros/humble/share/turtlebot3_gazebo/models'
        ),
        gzserver,
        robot_state_publisher,
        spawn_turtlebot,
    ])
