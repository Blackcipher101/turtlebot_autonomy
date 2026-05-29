#!/bin/bash
source /opt/ros/humble/setup.bash
export DISPLAY=:1

echo "[1] Static TFs"
ros2 run tf2_ros static_transform_publisher --frame-id base_footprint --child-frame-id base_link &
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id sim_lidar --z 0.18 &
sleep 2

echo "[2] odom->base_footprint bridge"
python3 /workspace/turtlebot3-autonomy/isaac_sim/odom_to_tf.py &
sleep 3

echo "[3] SLAM Toolbox"
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=false &

echo "Waiting for SLAM to produce a map..."
until ros2 topic echo /map --once 2>/dev/null | grep -q 'width:'; do
    sleep 2
done
echo "Map received — starting Nav2"

echo "[4] Nav2 A*"
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    params_file:=/workspace/turtlebot3-autonomy/isaac_sim/nav2_slam_astar.yaml &

echo "Waiting for Nav2 to activate..."
until ros2 service call /lifecycle_manager_navigation/is_active lifecycle_msgs/srv/GetState 2>/dev/null | grep -q 'label: active'; do
    sleep 2
done
echo "Nav2 is ACTIVE"

echo "[5] RViz"
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=false &

wait
