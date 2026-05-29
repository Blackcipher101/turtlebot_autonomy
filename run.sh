#!/bin/bash
# m-explore-ros2 launcher
# Usage:
#   ./run.sh build    — build the Docker image (~15 min first time)
#   ./run.sh sim      — terminal 1: Gazebo + robot + SLAM + Nav2
#   ./run.sh explore  — terminal 2: autonomous frontier exploration
#   ./run.sh shell    — bash in container
#   ./run.sh stop     — stop container

set -e
cd "$(dirname "$0")"
COMPOSE="docker compose -f .devcontainer/docker-compose.yml"
xhost +local:docker 2>/dev/null || true

case "${1:-help}" in
  build)
    $COMPOSE build
    ;;

  sim)
    $COMPOSE up -d
    # Build workspace if not already built
    docker exec m_explore_ros2 bash -c "
      source /opt/ros/humble/setup.bash
      cd /workspace/ros_ws
      [ -f install/setup.bash ] || colcon build --symlink-install \
        --packages-select explore_lite_msgs explore_lite \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    "
    # Step 1: Gazebo + TB3 burger in house world (headless — no gzclient)
    docker exec -d m_explore_ros2 bash -c "
      source /opt/ros/humble/setup.bash
      export TURTLEBOT3_MODEL=burger
      export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:\$GAZEBO_MODEL_PATH
      export DISPLAY=:1 QT_X11_NO_MITSHM=1
      export LIBGL_ALWAYS_SOFTWARE=1
      ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
        x_pose:=-1.0 y_pose:=-1.0 2>&1 | tee /tmp/gazebo.log
    "
    # Wait for robot to spawn
    until docker exec m_explore_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -q /scan"; do sleep 2; done
    echo "Robot spawned, /scan live"
    # Step 2: SLAM + Nav2
    docker exec -d m_explore_ros2 bash -c "
      source /opt/ros/humble/setup.bash
      export TURTLEBOT3_MODEL=burger
      export DISPLAY=:1 QT_X11_NO_MITSHM=1
      > /tmp/nav2.log
      ros2 launch nav2_bringup bringup_launch.py \
        slam:=True \
        use_sim_time:=True \
        2>&1 | tee /tmp/nav2.log
    "
    until docker exec m_explore_ros2 grep -q 'Managed nodes are active' /tmp/nav2.log 2>/dev/null; do sleep 3; done
    echo "Nav2 + SLAM active — run './run.sh explore' in another terminal"
    ;;

  explore)
    docker exec -it m_explore_ros2 bash -c "
      source /opt/ros/humble/setup.bash
      source /workspace/ros_ws/install/setup.bash
      export TURTLEBOT3_MODEL=burger
      ros2 launch explore_lite explore.launch.py
    "
    ;;

  shell)
    $COMPOSE up -d
    docker exec -it m_explore_ros2 bash
    ;;

  stop)
    $COMPOSE down
    ;;

  help|*)
    echo "Usage: ./run.sh [build|sim|explore|shell|stop]"
    ;;
esac
