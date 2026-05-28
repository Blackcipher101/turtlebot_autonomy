# CLAUDE.md — TurtleBot3 Semantic Autonomy Stack

This file documents everything a future Claude session needs to understand, continue, or debug this project.

---

## What This Is

A complete ROS2 Humble autonomous robot stack running in Docker on macOS Apple Silicon:
- **Gazebo Classic 11** simulation (office world with 4 semantic zones)
- **SLAM Toolbox** (online sync) for mapping
- **Nav2** for path planning and obstacle avoidance
- **explore_lite** (m-explore-ros2) for autonomous frontier exploration
- **RViz2** rendered via Xvfb + x11vnc (VNC on port 5900)
- Semantic perception pipeline (Grounding-DINO + CLIP + NetworkX) — partially implemented

---

## Quick Start

```bash
# Build image (first time, ~15-20 min)
cd ~/turtlebot3-autonomy
docker compose -f docker/docker-compose.yml build

# Run the full stack
docker compose -f docker/docker-compose.yml up -d
docker exec -it tb3_sim /ros2_ws/scripts/start_vnc.sh

# Connect VNC viewer to localhost:5900 (no password)
# Watch the robot autonomously map the office
```

---

## Architecture

```
Xvfb :99 → x11vnc (port 5900)
     ↓
gzserver + office_semantic.world
     ↓ /scan, /odom, /tf
robot_state_publisher
     ↓
slam_toolbox (online_sync) → /map, /map_updates
     ↓
Nav2 (costmaps + NavfnPlanner + DWB) → /cmd_vel
     ↑
explore_lite → NavigateToPose actions
     ↓
RViz2 (autonomy.rviz)
```

---

## Key File Locations

| File | Purpose |
|------|---------|
| `scripts/start_vnc.sh` | Master launch script — start everything here |
| `configs/nav2_params.yaml` | Nav2 full config (costmaps, planner, DWB) |
| `configs/slam_toolbox_params.yaml` | SLAM Toolbox params |
| `src/m-explore-ros2/explore/config/params.yaml` | explore_lite params (hardcoded by its launch file) |
| `configs/rviz/autonomy.rviz` | RViz2 display config |
| `src/turtlebot3_sim/worlds/office_semantic.world` | Gazebo office world |
| `docker/Dockerfile` | Docker image definition |
| `docker/docker-compose.yml` | Container config (volumes, ports) |

---

## Critical Lessons Learned (In Order of Pain)

### 1. SLAM Map Header `sec: 10` Does NOT Mean SLAM Is Frozen

The `/map` topic message has two timestamp fields:
```
header.stamp.sec: <last_update_time>   # this changes as map grows
map_load_time.sec: 10                   # this is fixed at initialization
```
`map_load_time` is always `sec: 10` (Gazebo sim time at SLAM startup). It is **not** the last scan processed. If the map resolution/cells are growing in RViz, SLAM is working. Do not restart just because `sec: 10` appears in the raw topic echo.

### 2. explore_lite Params File Is NOT Configurable via Launch Args

`explore_lite`'s launch file (`explore.launch.py`) hardcodes the config path to its own package's `config/params.yaml`. There is no `params_file` argument. To change any explore_lite parameter, edit:
```
src/m-explore-ros2/explore/config/params.yaml
```
Then rebuild the workspace inside the container:
```bash
docker exec tb3_sim bash -c "cd /ros2_ws && colcon build --packages-select explore_lite --symlink-install"
```

### 3. turtlebot3_navigation2 Always Launches Its Own RViz2

The TB3 navigation2 launch file unconditionally starts a RViz2 instance with `tb3_navigation2.rviz`. This eats ~125% CPU inside Docker, starving Gazebo physics (robot stops moving). Always kill it after Nav2 activates:
```bash
pkill -f "tb3_navigation2.rviz" 2>/dev/null || true
```
This is already in `start_vnc.sh`.

### 4. Global Costmap Starts Tiny (126×80) — explore_lite Needs Full Size

When Nav2 first activates, `slam_toolbox` hasn't finished initializing the full map. The global costmap starts at ~126×80 cells. At this size, every frontier is within the goal tolerance, so explore_lite immediately reports "All frontiers traversed" and stops.

The fix is to wait until the global costmap reaches a meaningful size before starting explore_lite:
```bash
while [ $WAIT -lt 40 ]; do
  MAP_W=$(ros2 topic echo /global_costmap/costmap --once 2>/dev/null | grep -m1 'width:' | awk '{print $2}')
  [ "${MAP_W:-0}" -ge "200" ] && break
  sleep 4; WAIT=$((WAIT+4))
done
```
Full map is typically 384×384. This wait is already in `start_vnc.sh`.

### 5. QoS Mismatch: SLAM Toolbox RELIABLE vs Gazebo BEST_EFFORT

SLAM Toolbox's `/scan` subscriber defaults to RELIABLE QoS. Gazebo's `/scan` publisher uses BEST_EFFORT. In ROS2, RELIABLE subscribers cannot receive from BEST_EFFORT publishers.

**Do NOT add QoS override parameters to `slam_toolbox_params.yaml`** — the slam_toolbox parameter names for QoS are wrong/unsupported and cause SLAM to process only the very first scan and then silently stop.

The fix used: switch to `online_sync_launch.py` instead of `online_async_launch.py`. The sync node handles QoS negotiation differently and processes scans continuously.

### 6. Robot Velocity: Never Publish to /cmd_vel Directly During Nav2

Nav2's `velocity_smoother` node intercepts and overrides any direct `/cmd_vel` publishes. Attempting to spin the robot in place by publishing to `/cmd_vel` before/during Nav2 operation doesn't work — the smoother resets it.

To build an initial map before explore_lite, set `minimum_travel_distance: 0.0` and `minimum_travel_heading: 0.0` in `slam_toolbox_params.yaml`. SLAM then accepts every scan regardless of motion.

### 7. Navigation Failures Near Inflated Obstacles

The meeting room chairs caused repeated `Planner failed` errors because frontier goals landed inside the inflated obstacle zone. Fixes applied:
- `NavfnPlanner tolerance: 2.0` — planner accepts goal within 2m of requested point
- `inflation_radius: 0.15` — reduced from 0.20 (TB3 burger is only 0.178m wide)
- `robot_radius: 0.12` in both costmaps

### 8. Container OOM (Exit Code 137)

Running gzclient (Gazebo GUI) + RViz2 + SLAM + Nav2 + Gazebo physics simultaneously exceeds available Docker memory on typical configurations.

**Solution:** Remove gzclient entirely. Use only VNC → RViz2 for visualization. The `start_vnc.sh` no longer launches gzclient. RViz2 displays the map, robot model, laser scan, and planned path — sufficient for monitoring.

### 9. Correct Launch Order (Timing Matters)

Wrong order causes TF lookup failures, costmap crashes, and Nav2 refusing to activate. The correct sequence with wait conditions:

```
1. Xvfb → sleep 2 → openbox → sleep 1
2. x11vnc
3. gzserver → wait until /spawn_entity service appears
4. spawn_entity (TB3 burger at 1.0, -1.5)
5. robot_state_publisher
6. wait until /scan appears
7. slam_toolbox (online_sync) → sleep 5
8. Nav2 → wait until "Managed nodes are active" in log
9. pkill tb3_navigation2.rviz
10. publish /initialpose → sleep 2
11. wait until global costmap width ≥ 200 cells (up to 40s)
12. explore_lite
13. RViz2
```

### 10. explore_lite Uses /map Not /global_costmap/costmap

Despite `costmap_topic: map` in params, explore_lite subscribes to the raw `/map` OccupancyGrid from SLAM (not the Nav2 costmap). `costmap_updates_topic: map_updates` is the incremental update topic from slam_toolbox. Both must be publishing before explore_lite finds frontiers.

---

## Current Configuration Summary

### SLAM (`configs/slam_toolbox_params.yaml`)
```yaml
mode: mapping
use_sim_time: true
resolution: 0.05
max_laser_range: 3.5          # TB3 burger LiDAR physical max
minimum_time_interval: 0.5
minimum_travel_distance: 0.0  # accept every scan regardless of movement
minimum_travel_heading: 0.0
transform_timeout: 2.0
tf_buffer_duration: 120.0
```

### Nav2 (`configs/nav2_params.yaml`)
- Planner: `NavfnPlanner` with A*, `tolerance: 2.0`, `allow_unknown: true`
- Local planner: DWB, `max_vel_x: 0.18`, `max_vel_theta: 0.8`
- `transform_tolerance: 5.0` everywhere (sim_time lag tolerance)
- `inflation_radius: 0.15`, `robot_radius: 0.12`

### explore_lite (`src/m-explore-ros2/explore/config/params.yaml`)
```yaml
robot_base_frame: base_link
costmap_topic: map
costmap_updates_topic: map_updates
planner_frequency: 0.5
progress_timeout: 25.0
min_frontier_size: 0.5
transform_tolerance: 2.0
```

---

## Docker Details

- **Image:** `turtlebot3-autonomy:humble` (~9GB uncompressed, ~2GB compressed)
- **Base:** `osrf/ros:humble-desktop-full` (linux/arm64, native Apple Silicon)
- **Named volumes:** `tb3_install`, `tb3_build` (persistent colcon build artifacts)
- **Ports:** `5900:5900` (VNC), `11345:11345` (Gazebo)
- **Key env vars:**
  ```
  TURTLEBOT3_MODEL=burger
  ROS_DOMAIN_ID=0
  DISPLAY=:99
  GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
  ```

---

## Workspace Build (Inside Container)

```bash
docker exec tb3_sim bash -c "
  source /opt/ros/humble/setup.bash
  cd /ros2_ws
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tail -20
"
```

Packages that need rebuilding after changes:
- `explore_lite` — after editing `src/m-explore-ros2/explore/config/params.yaml`
- `semantic_msgs` — must build first (other semantic packages depend on it)
- `rrt_planner` — C++ plugin, needs full cmake build

---

## What's Not Yet Complete

The following phases from the original plan are scaffolded but not fully tested:

| Component | Status | Notes |
|-----------|--------|-------|
| SLAM + exploration | Working | Core loop functional |
| RRT* planner | Scaffolded | C++ plugin in `src/rrt_planner/`; NavfnPlanner used as stand-in |
| Semantic perception | Scaffolded | `grounding_dino_wrapper.py`, `clip_embedder.py` exist; not launched |
| Semantic graph | Scaffolded | NetworkX graph server in `src/semantic_graph/`; not tested |
| Semantic navigation | Scaffolded | Query node + nav client in `src/semantic_navigation/` |
| RViz semantic markers | Scaffolded | `src/semantic_viz/` |

To continue semantic pipeline work: build semantic_msgs first, then launch perception node after exploration is complete.

---

## Troubleshooting

**Robot not moving / frontiers immediately exhausted:**
- Check costmap size: `ros2 topic echo /global_costmap/costmap --once | grep width`
- Should be ≥200 before explore_lite starts. Wait up to 40s.

**SLAM map frozen:**
- Check `/scan` is publishing: `ros2 topic hz /scan`
- Check TF: `ros2 run tf2_tools view_frames` — `odom→base_footprint` must exist
- Check slam log: `docker exec tb3_sim cat /tmp/slam.log`

**explore_lite "All frontiers traversed" immediately:**
- Kill and restart after costmap grows: the start_vnc.sh wait loop handles this
- If it still happens, check `min_frontier_size` in `src/m-explore-ros2/explore/config/params.yaml`

**Nav2 "Planner failed" repeatedly:**
- Frontier goal is inside inflated obstacle
- Increase `NavfnPlanner tolerance` or reduce `inflation_radius`
- Current setting (tolerance: 2.0) should handle most cases

**VNC black screen:**
- Check Xvfb: `docker exec tb3_sim ps aux | grep Xvfb`
- Reconnect — VNC is on `localhost:5900`, no password

**Container exits with code 137:**
- OOM — increase Docker Desktop memory limit (recommend 10GB+)
- Ensure gzclient is NOT running (it's removed from start_vnc.sh)

**"transform_timeout" warnings in Nav2:**
- Normal during startup. `transform_tolerance: 5.0` in nav2_params.yaml suppresses most failures.
- If robot never starts navigating, wait 30s after Nav2 activates.
