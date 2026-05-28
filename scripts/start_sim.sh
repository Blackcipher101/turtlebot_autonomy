#!/bin/bash
# Start headless Gazebo + TurtleBot3 inside Docker container
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}

WORLD=/ros2_ws/src/turtlebot3_sim/worlds/office_semantic.world
BURGER_SDF=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
URDF=/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf

echo "=== Starting gzserver ==="
# Unset DISPLAY so gzserver skips render engine init (pure headless)
unset DISPLAY
# -s loads system plugins: init (clock/params) and factory (/spawn_entity)
gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  "$WORLD" &
GZSERVER_PID=$!

echo "=== Starting robot_state_publisher ==="
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=true \
  -p robot_description:="$(xacro $URDF)" &
RSP_PID=$!

echo "=== Waiting for /spawn_entity service ==="
until ros2 service list 2>/dev/null | grep -q spawn_entity; do
  echo "  waiting for gzserver to load plugins..."
  sleep 2
done

echo "=== Spawning TurtleBot3 Burger ==="
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file "$BURGER_SDF" \
  -x 1.0 -y -1.5 -z 0.01

echo "=== Simulation running. Topics: ==="
ros2 topic list

# Keep alive
wait $GZSERVER_PID
