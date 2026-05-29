"""
Isaac Sim simulation: TurtleBot3 Burger + ROS2 bridge — Office environment.

Publishes:  /scan (LaserScan), /odom (Odometry), /tf,
            /depth (Image), /rgb (Image), /camera_info (CameraInfo)
Subscribes: /cmd_vel (Twist)

Run:
  ISAACSIM_ACCEPT_EULA=YES DISPLAY=:1 /mnt/data/isaac_sim_env/bin/python \
    /home/phiserver/turtlebot3-autonomy/isaac_sim/turtlebot3_ros2_sim.py
"""

import ctypes
ctypes.CDLL(
    "/mnt/data/isaac_sim_env/lib/python3.10/site-packages/nvidia/nccl/lib/libnccl.so.2",
    ctypes.RTLD_GLOBAL,
)

import isaacsim  # must be first
from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False, "hide_ui": False, "width": 1280, "height": 720})

# ── Imports after SimulationApp ───────────────────────────────────────────────
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import usdrt.Sdf
from pxr import Gf, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import create_prim
from isaacsim.robot.wheeled_robots.robots.wheeled_robot import WheeledRobot

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ── World ─────────────────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
world.scene.add_default_ground_plane()

# ── Office environment ────────────────────────────────────────────────────────
create_prim(
    prim_path="/World/Office",
    prim_type="Xform",
    usd_path=(
        "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
        "/Assets/Isaac/4.5/Isaac/Environments/Office/office.usd"
    ),
    position=np.array([0.0, 0.0, 0.0]),
)

# ── TurtleBot3 Burger ─────────────────────────────────────────────────────────
robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/TurtleBot3",
        name="turtlebot3",
        wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
        create_robot=True,
        usd_path=(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
            "/Assets/Isaac/4.5/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
        ),
        position=np.array([0.0, 0.0, 0.01]),
    )
)

world.reset()
# Let stage fully settle before creating sensors
for _ in range(10):
    simulation_app.update()

# ── RTX Lidar (created after stage is settled) ────────────────────────────────
_, lidar_sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/TurtleBot3/lidar",
    parent="/World/TurtleBot3",
    config="Example_Rotary",
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)
simulation_app.update()

# ── Depth Camera (front-facing) ───────────────────────────────────────────────
camera_prim = create_prim(
    prim_path="/World/TurtleBot3/depth_camera",
    prim_type="Camera",
    position=np.array([0.08, 0.0, 0.15]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
)
from pxr import UsdGeom as UG
camera = UG.Camera.Get(world.stage, "/World/TurtleBot3/depth_camera")
camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
camera.GetFocalLengthAttr().Set(1.93)
simulation_app.update()

# ── OmniGraph: ROS2 bridge ────────────────────────────────────────────────────
keys = og.Controller.Keys
LIDAR_PATH = lidar_sensor.GetPath().pathString
CAMERA_PATH = "/World/TurtleBot3/depth_camera"

lidar_rp = rep.create.render_product(LIDAR_PATH, resolution=[1, 1])
lidar_render_product = lidar_rp.path
simulation_app.update()

og.Controller.edit(
    {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
            ("SimTime",           "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ROS2Context",       "isaacsim.ros2.bridge.ROS2Context"),
            # /cmd_vel → differential drive → wheels
            ("SubCmdVel",         "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("BreakLinVel",       "omni.graph.nodes.BreakVector3"),
            ("BreakAngVel",       "omni.graph.nodes.BreakVector3"),
            ("DiffCtrl",          "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArtCtrl",           "isaacsim.core.nodes.IsaacArticulationController"),
            # Odometry
            ("ComputeOdom",       "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom",           "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            # TF: odom->base_footprint via external odom_to_tf.py node (avoids dual-parent conflict)
            # RTX Lidar → /scan + /point_cloud
            ("LidarScan",         "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ("LidarPCL",          "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            # Depth camera render product + publishers
            ("CreateRenderProd",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("PubRGB",            "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PubDepth",          "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PubCamInfo",        "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.SET_VALUES: [
            ("ROS2Context.inputs:domain_id",              0),
            ("SubCmdVel.inputs:topicName",                "/cmd_vel"),
            # TurtleBot3 Burger: wheel radius 0.033m, wheel separation 0.16m
            ("DiffCtrl.inputs:wheelRadius",               0.033),
            ("DiffCtrl.inputs:wheelDistance",             0.16),
            ("ArtCtrl.inputs:jointNames",                 ["wheel_left_joint", "wheel_right_joint"]),
            ("ArtCtrl.inputs:robotPath",                  "/World/TurtleBot3"),
            ("ComputeOdom.inputs:chassisPrim",            "/World/TurtleBot3"),
            ("PubOdom.inputs:topicName",                  "/odom"),
            ("PubOdom.inputs:chassisFrameId",             "base_footprint"),
            # Lidar
            ("LidarScan.inputs:renderProductPath",        lidar_render_product),
            ("LidarScan.inputs:topicName",                "/scan"),
            ("LidarScan.inputs:type",                     "laser_scan"),
            ("LidarScan.inputs:resetSimulationTimeOnStop", True),
            ("LidarPCL.inputs:renderProductPath",         lidar_render_product),
            ("LidarPCL.inputs:topicName",                 "/point_cloud"),
            ("LidarPCL.inputs:type",                      "point_cloud"),
            ("LidarPCL.inputs:resetSimulationTimeOnStop", True),
            # Depth camera
            ("CreateRenderProd.inputs:cameraPrim",        [usdrt.Sdf.Path(CAMERA_PATH)]),
            ("CreateRenderProd.inputs:width",             640),
            ("CreateRenderProd.inputs:height",            480),
            ("PubRGB.inputs:topicName",                   "/rgb"),
            ("PubRGB.inputs:type",                        "rgb"),
            ("PubDepth.inputs:topicName",                 "/depth"),
            ("PubDepth.inputs:type",                      "depth"),
            ("PubCamInfo.inputs:topicName",               "/camera_info"),
            ("PubCamInfo.inputs:type",                    "camera_info"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick",           "SubCmdVel.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",           "ArtCtrl.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",           "ComputeOdom.inputs:execIn"),
            ("SubCmdVel.outputs:execOut",             "DiffCtrl.inputs:execIn"),
            ("SubCmdVel.outputs:linearVelocity",      "BreakLinVel.inputs:tuple"),
            ("BreakLinVel.outputs:x",                 "DiffCtrl.inputs:linearVelocity"),
            ("SubCmdVel.outputs:angularVelocity",     "BreakAngVel.inputs:tuple"),
            ("BreakAngVel.outputs:z",                 "DiffCtrl.inputs:angularVelocity"),
            ("DiffCtrl.outputs:velocityCommand",      "ArtCtrl.inputs:velocityCommand"),
            ("OnPlaybackTick.outputs:tick",           "LidarScan.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",           "LidarPCL.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick",           "CreateRenderProd.inputs:execIn"),
            ("ComputeOdom.outputs:execOut",           "PubOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position",          "PubOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation",       "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity",    "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity",   "PubOdom.inputs:angularVelocity"),
            ("SimTime.outputs:simulationTime",        "PubOdom.inputs:timeStamp"),
            ("CreateRenderProd.outputs:execOut",      "PubRGB.inputs:execIn"),
            ("CreateRenderProd.outputs:execOut",      "PubDepth.inputs:execIn"),
            ("CreateRenderProd.outputs:execOut",      "PubCamInfo.inputs:execIn"),
            ("CreateRenderProd.outputs:renderProductPath", "PubRGB.inputs:renderProductPath"),
            ("CreateRenderProd.outputs:renderProductPath", "PubDepth.inputs:renderProductPath"),
            ("CreateRenderProd.outputs:renderProductPath", "PubCamInfo.inputs:renderProductPath"),
            ("ROS2Context.outputs:context",           "SubCmdVel.inputs:context"),
            ("ROS2Context.outputs:context",           "PubOdom.inputs:context"),
            ("ROS2Context.outputs:context",           "LidarScan.inputs:context"),
            ("ROS2Context.outputs:context",           "LidarPCL.inputs:context"),
            ("ROS2Context.outputs:context",           "PubRGB.inputs:context"),
            ("ROS2Context.outputs:context",           "PubDepth.inputs:context"),
            ("ROS2Context.outputs:context",           "PubCamInfo.inputs:context"),
        ],
    },
)

world.play()
print("[Isaac Sim] TurtleBot3 in Office — scene ready.")
print("[Isaac Sim] Publishing: /scan  /point_cloud  /odom  /tf  /rgb  /depth  /camera_info")
print("[Isaac Sim] Subscribing: /cmd_vel")

try:
    while simulation_app.is_running():
        world.step(render=True)
except KeyboardInterrupt:
    print("[Isaac Sim] Shutting down.")
finally:
    simulation_app.close()
