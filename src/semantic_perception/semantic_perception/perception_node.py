"""Semantic perception node — sparse trigger, async inference."""

from __future__ import annotations
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from semantic_msgs.msg import SemanticObject, SemanticSnapshot
from .grounding_dino_wrapper import GroundingDINOWrapper
from .room_inference import RoomInferenceEngine


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('semantic_perception_node')

        self.declare_parameter('min_travel_distance', 1.5)   # metres
        self.declare_parameter('periodic_interval',   30.0)  # seconds
        self.declare_parameter('dino_confidence',     0.35)

        self._min_dist = self.get_parameter('min_travel_distance').value
        self._interval = self.get_parameter('periodic_interval').value
        dino_conf      = self.get_parameter('dino_confidence').value

        self._dino     = GroundingDINOWrapper(confidence_threshold=dino_conf)
        self._room_eng = RoomInferenceEngine()

        self._last_image: np.ndarray | None = None
        self._last_pose:  tuple[float, float] | None = None
        self._last_inference_pose: tuple[float, float] | None = None
        self._last_inference_time: float = 0.0
        self._inference_lock = threading.Lock()
        self._inference_running = False

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.create_subscription(Image,    '/camera/image_raw', self._image_cb, qos_sensor)
        self.create_subscription(Odometry, '/odom',             self._odom_cb,  10)

        self._snap_pub = self.create_publisher(SemanticSnapshot, '/semantic_snapshot', 10)

        # Periodic timer as fallback trigger
        self.create_timer(5.0, self._timer_cb)

        self.get_logger().info('SemanticPerceptionNode ready (sparse trigger mode).')

    def _image_cb(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'bgr8':
                arr = arr[:, :, ::-1].copy()
            elif msg.encoding in ('mono8', 'bayer_rggb8'):
                arr = np.stack([arr[:, :, 0]] * 3, axis=-1)
            self._last_image = arr
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._last_pose = (x, y)
        self._maybe_trigger()

    def _timer_cb(self):
        self._maybe_trigger()

    def _maybe_trigger(self):
        if self._last_image is None or self._last_pose is None:
            return
        if self._inference_running:
            return

        now = time.time()
        elapsed = now - self._last_inference_time

        # Trigger: distance threshold
        distance_ok = False
        if self._last_inference_pose is None:
            distance_ok = True
        else:
            dx = self._last_pose[0] - self._last_inference_pose[0]
            dy = self._last_pose[1] - self._last_inference_pose[1]
            distance_ok = math.hypot(dx, dy) >= self._min_dist

        # Trigger: time threshold
        time_ok = elapsed >= self._interval

        if distance_ok or time_ok:
            self._run_inference_async(self._last_image.copy(), self._last_pose)

    def _run_inference_async(self, image: np.ndarray, pose: tuple[float, float]):
        self._inference_running = True

        def _worker():
            try:
                self._run_inference(image, pose)
            finally:
                self._inference_running = False

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _run_inference(self, image: np.ndarray, pose: tuple[float, float]):
        self.get_logger().info(f'Running semantic inference at ({pose[0]:.1f}, {pose[1]:.1f})')

        detections = self._dino.detect(image)
        room_label, room_conf = self._room_eng.infer(
            [(d[0], d[1]) for d in detections]
        )

        snap = SemanticSnapshot()
        snap.header.stamp = self.get_clock().now().to_msg()
        snap.header.frame_id = 'map'
        snap.room_label = room_label
        snap.room_confidence = float(room_conf)
        snap.robot_pose.position.x = pose[0]
        snap.robot_pose.position.y = pose[1]
        snap.robot_pose.orientation.w = 1.0

        for label, conf, bbox in detections:
            obj = SemanticObject()
            obj.label = label
            obj.confidence = float(conf)
            obj.position = Point(x=pose[0], y=pose[1], z=0.0)
            obj.bbox = [float(v) for v in bbox]
            snap.objects.append(obj)

        self._snap_pub.publish(snap)
        self._last_inference_pose = pose
        self._last_inference_time = time.time()

        self.get_logger().info(
            f'Snapshot published: room={room_label} ({room_conf:.2f}), '
            f'{len(detections)} objects'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
