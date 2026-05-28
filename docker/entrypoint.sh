#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null || true

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-0}

exec "$@"
