# TurtleBot3 Semantic Autonomy Stack

> ROS2 Humble · Gazebo 11 · macOS Apple Silicon · Docker

Complete simulation stack for autonomous exploration, SLAM, custom RRT planning, and semantic navigation on TurtleBot3 Burger.

---

## Architecture

```
macOS (Apple Silicon)
└── Docker (Ubuntu 22.04 ARM64)
    └── ROS2 Humble
        ├── turtlebot3_gazebo  ──── Simulation + LiDAR + Camera
        ├── slam_toolbox       ──── Online async SLAM
        ├── explore_lite       ──── Frontier-based exploration
        ├── nav2_bringup       ──── Navigation stack
        ├── rrt_planner        ──── Custom RRT* global planner (C++)
        ├── semantic_perception──── Grounding-DINO + CLIP (sparse)
        ├── semantic_graph     ──── NetworkX semantic memory
        ├── semantic_navigation──── Text query → Nav2 goal
        └── semantic_viz       ──── RViz MarkerArray overlays
```

---

## Prerequisites

### 1. macOS Setup (run once)

```bash
chmod +x scripts/setup_mac.sh
./scripts/setup_mac.sh
```

What it does:
- Installs Homebrew, XQuartz, Docker Desktop
- Configures XQuartz for network connections (needed for GUI forwarding)

After install, **log out and log back in**, then:

```bash
# Open XQuartz
open -a XQuartz

# In XQuartz → Preferences → Security → check "Allow connections from network clients"

# Allow localhost
xhost +localhost

# Add to ~/.zshrc:
export DISPLAY=:0
```

### 2. Docker Desktop Settings

Open Docker Desktop → Settings:
- **Resources → Memory**: 8 GB minimum (12 GB recommended)
- **Resources → CPUs**: 4+
- **General**: Enable "Use Rosetta for x86/amd64 emulation" (optional fallback)

---

## Quick Start (3 commands)

```bash
# 1. Build the Docker image (once, ~15-20 min first time)
cd docker && docker compose build

# 2. Start container
docker compose run --rm ros2

# 3. Inside container: build workspace + launch full demo
bash /ros2_ws/scripts/build_workspace.sh
bash /ros2_ws/scripts/launch_demo.sh full
```

---

## Step-by-Step Launch

Each step in a separate terminal tab (`docker exec -it tb3_autonomy bash`):

### Step 1 — Simulation

```bash
source /ros2_ws/install/setup.bash
ros2 launch turtlebot3_sim sim.launch.py
```

Verify:
```bash
ros2 topic list | grep -E 'scan|odom|camera'
ros2 run tf2_tools view_frames
```

### Step 2 — SLAM + Autonomous Exploration

```bash
ros2 launch slam_exploration slam_explore.launch.py
```

The robot will autonomously explore and build a map. Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/office_map
```

### Step 3 — RViz

```bash
ros2 launch turtlebot3_sim rviz.launch.py
```

### Step 4 — Semantic Stack

```bash
# Perception (Grounding-DINO + CLIP — loads ~30s first time)
ros2 launch semantic_perception perception.launch.py

# Semantic graph server
ros2 launch semantic_graph graph_server.launch.py

# Semantic navigation query node
ros2 launch semantic_navigation semantic_nav.launch.py

# RViz markers
ros2 launch semantic_viz viz.launch.py
```

### Step 5 — Send a Semantic Navigation Command

```bash
ros2 service call /navigate_semantic semantic_msgs/srv/SemanticQuery \
  "{query_text: 'Go to the kitchen'}"

ros2 service call /navigate_semantic semantic_msgs/srv/SemanticQuery \
  "{query_text: 'Navigate to the pantry'}"

ros2 service call /navigate_semantic semantic_msgs/srv/SemanticQuery \
  "{query_text: 'Where is the meeting room?'}"
```

---

## Teleop (manual control)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Custom RRT Planner

The RRT* planner is registered as a Nav2 `GlobalPlanner` plugin. It activates automatically when Nav2 uses the `GridBased` planner (configured in `configs/nav2_params.yaml`).

To see the RRT tree in RViz: add **MarkerArray** display → topic `/rrt_tree`.

Parameters (editable in `configs/nav2_params.yaml`):
| Parameter | Default | Description |
|---|---|---|
| `max_iterations` | 5000 | Max RRT iterations per plan |
| `step_size` | 0.1 m | Tree expansion step |
| `goal_tolerance` | 0.2 m | Distance to declare goal reached |
| `goal_bias` | 0.05 | Probability of sampling goal directly |
| `enable_rrt_star` | true | Enable rewiring (RRT* vs RRT) |
| `rewire_radius` | 0.5 m | Rewire neighborhood radius |

---

## Semantic Navigation Details

The semantic pipeline:
1. Camera image → **Grounding-DINO-tiny** detects objects (sparse, every 1.5m or 30s)
2. Object list → **CLIP ViT-B/32** embeds scene description → infers room type
3. Room + objects → **NetworkX graph** (persisted to `/ros2_ws/maps/semantic_graph.json`)
4. Text query → CLIP embeds query → cosine similarity to known rooms → Nav2 goal

No hardcoded label→room mappings. All inference is embedding-based.

---

## Gazebo World

`office_semantic.world` contains four zones:
| Zone | Landmarks |
|---|---|
| Kitchen (top-left) | microwave, sink, refrigerator, tables |
| Pantry (top-right) | shelves, storage boxes |
| Meeting room (bottom-left) | round table, 4 chairs, whiteboard |
| Hallway | trash can, plant, sofa |

---

## Troubleshooting

### Gazebo black screen / RViz blank
```bash
# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
```
Or in `docker-compose.yml` set `LIBGL_ALWAYS_SOFTWARE: "1"`.

### XQuartz / display not working
```bash
# Check XQuartz is running
pgrep -x Xquartz

# Re-allow connections
xhost +localhost

# Verify DISPLAY
echo $DISPLAY  # should be :0
```

### explore_lite not found
```bash
# If apt package unavailable, build from source:
cd /ros2_ws/src
git clone -b humble https://github.com/robo-friends/m-explore-ros2.git
cd /ros2_ws && colcon build --packages-select explore_lite
```

### Nav2 not starting
```bash
# Check all lifecycle nodes are active
ros2 lifecycle list /bt_navigator
```

### Grounding-DINO slow on first inference
Expected. Model loads ~10-20s. Subsequent inferences: 3-8s on M2 CPU. This is by design — inference runs sparsely.

---

## Performance Expectations (Apple Silicon)

| Component | M1 | M2 | M3 |
|---|---|---|---|
| Gazebo FPS | 10-15 | 15-25 | 20-30 |
| SLAM update rate | 1 Hz | 2 Hz | 2 Hz |
| Grounding-DINO inference | 8-12s | 4-8s | 3-6s |
| CLIP inference | 1-2s | 0.5-1s | 0.3-0.8s |
| Nav2 planning (RRT*) | <1s | <0.5s | <0.3s |

---

## Project Structure

```
turtlebot3-autonomy/
├── docker/             # Dockerfile + docker-compose.yml
├── scripts/            # setup_mac.sh, build_workspace.sh, launch_demo.sh
├── .devcontainer/      # VSCode Dev Container config
├── configs/            # nav2_params.yaml, slam_toolbox_params.yaml, rviz/
├── maps/               # saved maps and semantic_graph.json (auto-generated)
└── src/
    ├── turtlebot3_sim/         # Gazebo world + launch wrappers
    ├── slam_exploration/       # slam_toolbox + explore_lite launch
    ├── rrt_planner/            # Custom C++ Nav2 RRT* plugin
    ├── semantic_msgs/          # Custom message/service definitions
    ├── semantic_perception/    # Grounding-DINO + CLIP + room inference
    ├── semantic_graph/         # NetworkX graph DB + ROS2 service
    ├── semantic_navigation/    # Query node + Nav2 client
    └── semantic_viz/           # RViz MarkerArray publisher
```
