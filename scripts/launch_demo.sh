#!/bin/bash
# One-shot demo launcher — runs inside the Docker container
# Usage: ./launch_demo.sh [mode]
# Modes: sim | explore | semantic | full

set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

MODE=${1:-full}

case "$MODE" in
  sim)
    echo "Launching simulation only..."
    ros2 launch turtlebot3_sim sim.launch.py
    ;;
  explore)
    echo "Launching SLAM + autonomous exploration..."
    ros2 launch slam_exploration slam_explore.launch.py
    ;;
  semantic)
    echo "Launching semantic stack (perception + graph + nav)..."
    ros2 launch semantic_perception perception.launch.py &
    ros2 launch semantic_graph graph_server.launch.py &
    ros2 launch semantic_navigation semantic_nav.launch.py &
    ros2 launch semantic_viz viz.launch.py
    ;;
  full)
    echo "Launching full autonomy stack..."
    echo "Starting simulation..."
    ros2 launch turtlebot3_sim sim.launch.py &
    sleep 5
    echo "Starting SLAM + exploration..."
    ros2 launch slam_exploration slam_explore.launch.py &
    sleep 3
    echo "Starting semantic stack..."
    ros2 launch semantic_perception perception.launch.py &
    ros2 launch semantic_graph graph_server.launch.py &
    ros2 launch semantic_navigation semantic_nav.launch.py &
    ros2 launch semantic_viz viz.launch.py &
    echo "Starting RViz..."
    ros2 launch turtlebot3_sim rviz.launch.py
    ;;
  *)
    echo "Unknown mode: $MODE"
    echo "Usage: $0 [sim|explore|semantic|full]"
    exit 1
    ;;
esac
