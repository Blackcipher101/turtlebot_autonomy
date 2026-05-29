# Isaac Sim — TurtleBot3 Autonomy Stack

## Overview

Replaces the Gazebo simulation with NVIDIA Isaac Sim 4.5 while keeping the full ROS2 Humble stack (SLAM Toolbox + Nav2 A*) running inside Docker.

```
Isaac Sim (host) → /scan /odom /tf /rgb /depth
        ↓  host network
Docker container (m_explore_ros2)
  ├── SLAM Toolbox  → /map
  ├── Nav2 A*       → /plan → /cmd_vel → Isaac Sim robot
  └── RViz2         → visualization + goal setting
```

---

## Installation

### Isaac Sim 4.5 (pip, Python 3.10)

```bash
conda create --prefix /mnt/data/isaac_sim_env python=3.10 -y
/mnt/data/isaac_sim_env/bin/pip install isaacsim==4.5.0 --extra-index-url https://pypi.nvidia.com
/mnt/data/isaac_sim_env/bin/pip install isaacsim[all]==4.5.0 --extra-index-url https://pypi.nvidia.com
```

**Environment:** `/mnt/data/isaac_sim_env` (~5.4GB)

### NCCL Fix (required)

torch 2.12.0 fails to import unless NCCL is pre-loaded with `RTLD_GLOBAL`. This is handled in the script:

```python
import ctypes
ctypes.CDLL(
    "/mnt/data/isaac_sim_env/lib/python3.10/site-packages/nvidia/nccl/lib/libnccl.so.2",
    ctypes.RTLD_GLOBAL,
)
```

---

## Running Isaac Sim

```bash
ISAACSIM_ACCEPT_EULA=YES DISPLAY=:1 \
  /mnt/data/isaac_sim_env/bin/python \
  /home/phiserver/turtlebot3-autonomy/isaac_sim/turtlebot3_ros2_sim.py
```

Takes ~2 min on first launch (shader compilation). Subsequent launches are fast (~20s).

---

## Isaac Sim Script: `turtlebot3_ros2_sim.py`

### What it sets up

| Component | Detail |
|-----------|--------|
| Environment | NVIDIA Office USD from Nucleus |
| Robot | TurtleBot3 Burger USD from Nucleus |
| Lidar | RTX Rotary Lidar (`Example_Rotary` config) |
| Camera | 640×480 depth camera, front-facing |
| ROS2 bridge | isaacsim.ros2.bridge extension |

### Published Topics

| Topic | Type | Source |
|-------|------|--------|
| `/scan` | LaserScan | RTX lidar |
| `/point_cloud` | PointCloud2 | RTX lidar |
| `/odom` | Odometry | IsaacComputeOdometry |
| `/tf` | TF | Robot prim tree |
| `/rgb` | Image | Depth camera |
| `/depth` | Image | Depth camera |
| `/camera_info` | CameraInfo | Depth camera |

### Subscribed Topics

| Topic | Type |
|-------|------|
| `/cmd_vel` | Twist (Nav2 → robot) |

### TF Frames

Isaac Sim publishes `world` as root frame (not `odom`). A static TF bridge is needed:

```bash
# Run inside Docker container
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id odom
```

---

## Docker Container: `m_explore_ros2`

The container runs on **host network** and sees all Isaac Sim ROS2 topics directly.

### Launch SLAM + Nav2 + RViz

```bash
docker exec m_explore_ros2 bash /workspace/isaac_sim/launch_slam_astar.sh
```

Or manually:

```bash
# 1. Static TF fix (world → odom)
docker exec -d m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id odom"

# 2. SLAM Toolbox
docker exec -d m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=false"

# 3. Nav2 with A* planner
docker exec -d m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    params_file:=/workspace/isaac_sim/nav2_slam_astar.yaml"

# 4. RViz
docker exec -d m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  export DISPLAY=:1
  ros2 launch nav2_bringup rviz_launch.py use_sim_time:=false"
```

### Nav2 Config: `nav2_slam_astar.yaml`

Key settings:
- `use_sim_time: false` — Isaac Sim doesn't publish `/clock`
- Global planner: `nav2_navfn_planner::NavfnPlanner` with `use_astar: true`
- Robot radius: `0.12m`
- Max velocity: `0.20 m/s`
- Inflation radius: `0.15m`

---

## Known Issues & Fixes

### 1. `use_sim_time: true` causes Nav2 to hang

Isaac Sim does NOT publish `/clock`. Always use `use_sim_time: false` in all Nav2 and SLAM params.

### 2. Nav2 shows "inactive" in RViz

Usually caused by missing `/clock`. After switching to `use_sim_time: false`, manually trigger activation:

```bash
docker exec m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes '{command: 1}'"
```

### 3. `map → odom` TF missing

Isaac Sim publishes `world → base_footprint` (not `odom → base_footprint`). Fix with static TF:

```bash
docker exec -d m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id odom"
```

### 4. RTX Lidar not publishing `/scan`

The lidar render product must be created **after** `world.reset()` and several `simulation_app.update()` calls, otherwise the TurtleBot3 USD isn't loaded yet. The fix in the script:

```python
world.reset()
for _ in range(10):
    simulation_app.update()
# Now create lidar and render product
_, lidar_sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", ...)
lidar_rp = rep.create.render_product(lidar_sensor.GetPath().pathString, resolution=[1, 1])
```

### 5. `CreateRenderProductCommand` not found

Wrong command name. Use `rep.create.render_product()` from `omni.replicator.core` instead.

### 6. NCCL `undefined symbol: ncclCommResume`

Fixed by pre-loading NCCL with `RTLD_GLOBAL` before importing `isaacsim`. See script header.

### 7. pkill kills both old and new Isaac Sim

The script name is the same on every launch. When restarting, `pkill -f turtlebot3_ros2_sim.py` kills ANY running instance including the one just launched. Always wait for the old process to fully die before launching a new one.

### 8. Duplicate slam/nav2 processes

Multiple launches stack up. Before relaunching, kill all:

```bash
docker exec m_explore_ros2 bash -c "
  pkill -f slam_toolbox
  pkill -f navigation_launch
  pkill -f rviz2
  pkill -f static_transform_publisher"
```

---

## Sending Nav Goals

**Via RViz:** Click "2D Goal Pose" → click on map

**Via CLI:**
```bash
docker exec m_explore_ros2 bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}'"
```

---

## File Locations

| File | Purpose |
|------|---------|
| `isaac_sim/turtlebot3_ros2_sim.py` | Main Isaac Sim script |
| `isaac_sim/nav2_slam_astar.yaml` | Nav2 params (A* planner, no sim time) |
| `isaac_sim/launch_slam_astar.sh` | Container launch script |
| `isaac_sim/ISAAC_SIM_SETUP.md` | This file |

---

## Quick Restart Procedure

```bash
# 1. Kill everything in container
docker exec m_explore_ros2 bash -c "pkill -f slam_toolbox; pkill -f navigation_launch; pkill -f rviz2; pkill -f static_transform_publisher"

# 2. Wait for Isaac Sim to be up (check: /scan publisher count > 0)
source /opt/ros/humble/setup.bash && ros2 topic info /scan | grep "Publisher count"

# 3. Relaunch nav stack
docker exec -d m_explore_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id odom"
docker exec -d m_explore_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=false"
docker exec -d m_explore_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=/workspace/isaac_sim/nav2_slam_astar.yaml"
docker exec -d m_explore_ros2 bash -c "source /opt/ros/humble/setup.bash && export DISPLAY=:1 && ros2 launch nav2_bringup rviz_launch.py use_sim_time:=false"
```
