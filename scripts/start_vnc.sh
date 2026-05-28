#!/bin/bash
# Full stack: Xvfb+VNC + Gazebo + SLAM + Nav2 + explore_lite + RViz2
# All nodes use sim_time=true — start order guarantees /clock is live before any node

set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null || true

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}
export DISPLAY=:99

echo "=== Starting Xvfb ==="
Xvfb :99 -screen 0 1280x1024x24 -ac +extension GLX +render -noreset &
sleep 2
openbox &
sleep 1

echo "=== Starting x11vnc on port 5900 ==="
x11vnc -display :99 -forever -nopw -rfbport 5900 -bg -quiet
echo "VNC ready — connect to localhost:5900"

echo "=== Starting gzserver ==="
gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  /ros2_ws/src/turtlebot3_sim/worlds/office_semantic.world &
GZSERVER_PID=$!

# Start gzclient only on Linux (macOS Docker lacks GPU passthrough — OOM risk)
if [ "$(uname)" = "Linux" ] && [ -z "$SKIP_GZCLIENT" ]; then
  echo "=== Starting gzclient ==="
  gzclient --verbose 2>&1 | tee /tmp/gzclient.log &
fi

echo "=== Waiting for /spawn_entity ==="
until ros2 service list 2>/dev/null | grep -q spawn_entity; do sleep 2; done

echo "=== Spawning TurtleBot3 at (1.0, -1.5) ==="
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 1.0 -y -1.5 -z 0.01

echo "=== Starting robot_state_publisher ==="
URDF=/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=true \
  -p robot_description:="$(xacro $URDF 2>/dev/null || cat $URDF)" &

echo "=== Waiting for /scan topic ==="
until ros2 topic list 2>/dev/null | grep -q '/scan'; do sleep 2; done
echo "Robot is live"

echo "=== Starting SLAM toolbox ==="
ros2 launch slam_toolbox online_sync_launch.py \
  use_sim_time:=true \
  slam_params_file:=/ros2_ws/configs/slam_toolbox_params.yaml \
  2>&1 | tee /tmp/slam.log &
SLAM_PID=$!
sleep 5

echo "=== Starting Nav2 ==="
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  params_file:=/ros2_ws/configs/nav2_params.yaml \
  2>&1 | grep -v rviz2 | tee /tmp/nav2.log &
NAV2_PID=$!

echo "=== Waiting for Nav2 to activate ==="
until grep -q 'Managed nodes are active' /tmp/nav2.log 2>/dev/null; do sleep 3; done
echo "Nav2 active"

echo "=== Killing TB3 default RViz2 (frees CPU for sim) ==="
pkill -f "tb3_navigation2.rviz" 2>/dev/null || true
sleep 2

echo "=== Setting initial pose ==="
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {position: {x: 1.0, y: -1.5, z: 0.0}, orientation: {w: 1.0}},
    covariance: [0.1,0,0,0,0,0, 0,0.1,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.05]
  }
}' 2>/dev/null || true
sleep 2

echo "=== Waiting for costmap to initialize ==="
# slam_toolbox expands the OccupancyGrid over time; Nav2 costmap must reach full size
# before explore_lite can find real frontiers (tiny 126x80 costmap yields no valid goals).
WAIT=0
while [ $WAIT -lt 40 ]; do
  MAP_W=$(ros2 topic echo /global_costmap/costmap --once 2>/dev/null | grep -m1 'width:' | awk '{print $2}')
  [ "${MAP_W:-0}" -ge "200" ] && break
  sleep 4; WAIT=$((WAIT+4))
done
echo "Costmap ready (${MAP_W:-unknown} cells wide)"

echo "=== Starting explore_lite ==="
ros2 launch explore_lite explore.launch.py \
  use_sim_time:=true \
  2>&1 | tee /tmp/explore.log &

echo "=== Starting RViz2 ==="
export QT_QPA_PLATFORM=xcb
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p /tmp/runtime-root
ros2 run rviz2 rviz2 \
  -d /ros2_ws/configs/rviz/autonomy.rviz \
  --ros-args -p use_sim_time:=true \
  2>&1 | tee /tmp/rviz.log &

echo ""
echo "========================================="
echo " Full stack running!"
echo " VNC: localhost:5900 (no password)"
echo " SLAM + Nav2 + explore_lite + RViz2 all up"
echo "========================================="

wait $GZSERVER_PID
