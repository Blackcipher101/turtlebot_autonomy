"""Publishes RViz MarkerArray for semantic objects and robot trajectory."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

from semantic_msgs.msg import SemanticSnapshot


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('semantic_viz_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(SemanticSnapshot, '/semantic_snapshot', self._snapshot_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)

        self._marker_pub = self.create_publisher(MarkerArray, '/semantic_markers', 10)
        self._traj_pub   = self.create_publisher(Marker,      '/robot_trajectory', 10)

        self._trajectory: list[Point] = []
        self._obj_markers: list[Marker] = []
        self._marker_id = 0

    def _odom_cb(self, msg: Odometry):
        p = Point()
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y
        p.z = 0.02
        self._trajectory.append(p)

        # Publish trajectory as LINE_STRIP
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'trajectory'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
        m.pose.orientation.w = 1.0
        m.points = self._trajectory[-500:]  # keep last 500 points
        self._traj_pub.publish(m)

    def _snapshot_cb(self, msg: SemanticSnapshot):
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=30)
        ma = MarkerArray()

        for obj in msg.objects:
            # Object sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'detected_objects'
            m.id = self._marker_id; self._marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = obj.position.x
            m.pose.position.y = obj.position.y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            # Color by confidence: green=high, red=low
            m.color = ColorRGBA(
                r=1.0 - obj.confidence,
                g=obj.confidence,
                b=0.0,
                a=0.8,
            )
            m.lifetime = lifetime
            ma.markers.append(m)

            # Object label text
            mt = Marker()
            mt.header = m.header
            mt.ns = 'object_labels'
            mt.id = self._marker_id; self._marker_id += 1
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = obj.position.x
            mt.pose.position.y = obj.position.y
            mt.pose.position.z = 0.35
            mt.pose.orientation.w = 1.0
            mt.scale.z = 0.2
            mt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            mt.text = f"{obj.label} ({obj.confidence:.2f})"
            mt.lifetime = lifetime
            ma.markers.append(mt)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
