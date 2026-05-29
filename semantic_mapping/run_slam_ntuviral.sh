#!/bin/bash
# Run Point-LIO on NTU VIRAL bag, save trajectory as TUM poses.
# Usage: ./run_slam_ntuviral.sh /mnt/data/eee_01.bag /mnt/data/semantic_output

BAG=${1:-"/mnt/data/eee_01.bag"}
OUTPUT=${2:-"/mnt/data/semantic_output"}
POSES_OUT="$OUTPUT/poses_tum.txt"

mkdir -p "$OUTPUT"

source /opt/ros/humble/setup.bash
source /home/phiserver/Point-LIO-ros2-robosense-Airy/install/setup.bash 2>/dev/null || \
  source /home/phiserver/lio-comparison/install/setup.bash 2>/dev/null || true

echo "[SLAM] Bag:    $BAG"
echo "[SLAM] Output: $OUTPUT"
echo "[SLAM] Poses:  $POSES_OUT"

# NTU VIRAL topics:
#   /os_cloud_node/points  (Ouster OS1-64 LiDAR)
#   /imu/imu               (IMU)
#   /cam0/image_raw        (camera)
#   /cam1/image_raw        (second camera)

# Launch Point-LIO with ouster config + pose saver
ros2 launch point_lio mapping_ouster64.launch.py \
  rviz:=false \
  &
SLAM_PID=$!
echo "[SLAM] Point-LIO PID: $SLAM_PID"

# Play bag
sleep 4
echo "[SLAM] Playing bag..."
ros2 bag play "$BAG" \
  --remap /os_cloud_node/points:=/cloud_registered \
  --clock &
BAG_PID=$!

# Save odometry to TUM format while running
echo "[SLAM] Saving poses to $POSES_OUT ..."
python3 - << 'EOF'
import sys, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os, time

OUTPUT = os.environ.get("POSES_OUT", "/mnt/data/semantic_output/poses_tum.txt")

class PoseSaver(Node):
    def __init__(self):
        super().__init__("pose_saver")
        self.f = open(OUTPUT, "w")
        self.f.write("# timestamp tx ty tz qx qy qz qw\n")
        self.count = 0
        self.create_subscription(Odometry, "/Odometry", self.cb, 100)
        self.get_logger().info(f"Saving poses to {OUTPUT}")

    def cb(self, msg):
        t  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        self.f.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                     f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
        self.count += 1
        if self.count % 100 == 0:
            self.f.flush()
            print(f"[PoseSaver] {self.count} poses saved")

rclpy.init()
node = PoseSaver()
rclpy.spin(node)
rclpy.shutdown()
EOF
POSES_OUT="$POSES_OUT"

wait $BAG_PID
sleep 5
kill $SLAM_PID 2>/dev/null

echo "[SLAM] Done. Poses saved to $POSES_OUT"
echo "[SLAM] Run pipeline:"
echo "  python run_pipeline.py --bag $BAG --lidar /os_cloud_node/points --camera /cam0/image_raw --poses $POSES_OUT"
