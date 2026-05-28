#!/bin/bash
# Run this INSIDE the Docker container to build the ROS2 workspace

set -e

source /opt/ros/humble/setup.bash

cd /ros2_ws

echo "Installing rosdep dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo "Building workspace..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select \
        semantic_msgs \
        rrt_planner \
        turtlebot3_sim \
        slam_exploration \
        semantic_perception \
        semantic_graph \
        semantic_navigation \
        semantic_viz

echo "Sourcing install..."
source install/setup.bash

echo "Build complete."
