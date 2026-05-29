#!/bin/bash
# Launch SLAM + A* Nav2 inside the Docker container, using Isaac Sim as the robot source.
# Isaac Sim publishes: /scan, /odom, /tf, /cmd_vel (subscribed)

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0

PARAMS=/workspace/isaac_sim/nav2_slam_astar.yaml

echo "[1/3] Killing any existing nav2/gazebo/slam processes..."
pkill -f "gzserver" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "nav2_bringup" 2>/dev/null
pkill -f "bt_navigator" 2>/dev/null
pkill -f "rrt_planner" 2>/dev/null
sleep 3

echo "[2/3] Starting SLAM Toolbox (online sync)..."
ros2 launch slam_toolbox online_sync_launch.py \
    use_sim_time:=true \
    slam_params_file:=/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml \
    &
sleep 4

echo "[3/3] Starting Nav2 (A* planner, no Gazebo)..."
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true \
    params_file:=$PARAMS \
    &

echo ""
echo "SLAM + A* Nav2 running."
echo "  /map        — SLAM map building"
echo "  /scan       — from Isaac Sim lidar"
echo "  /cmd_vel    — Nav2 → Isaac Sim robot"
echo ""
echo "To send a goal: ros2 topic pub /goal_pose geometry_msgs/PoseStamped ..."
echo "Press Ctrl+C to stop all."

wait
