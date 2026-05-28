"""ROS2 node: subscribes to SemanticSnapshot, exposes SemanticQuery service,
publishes MarkerArray for RViz visualization."""

import signal
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

from semantic_msgs.msg import SemanticSnapshot
from semantic_msgs.srv import SemanticQuery

from .graph_db import SemanticGraphDB


class GraphServerNode(Node):
    def __init__(self):
        super().__init__('semantic_graph_server')

        self.declare_parameter('graph_save_path', '/ros2_ws/maps/semantic_graph.json')
        save_path = self.get_parameter('graph_save_path').value

        self._db = SemanticGraphDB(save_path=save_path)

        self.create_subscription(
            SemanticSnapshot, '/semantic_snapshot', self._snapshot_cb, 10
        )

        self._query_srv = self.create_service(
            SemanticQuery, '/semantic_query', self._query_cb
        )

        self._marker_pub = self.create_publisher(
            MarkerArray, '/semantic_graph_markers', 10
        )

        # Publish markers every 5 s
        self.create_timer(5.0, self._publish_markers)

        # Save on shutdown
        signal.signal(signal.SIGTERM, lambda *_: self._save_and_exit())
        signal.signal(signal.SIGINT,  lambda *_: self._save_and_exit())

        self.get_logger().info('SemanticGraphServer ready.')

    def _snapshot_cb(self, msg: SemanticSnapshot):
        objects = [
            {
                'label': o.label,
                'confidence': o.confidence,
                'pose_x': o.position.x,
                'pose_y': o.position.y,
            }
            for o in msg.objects
        ]
        room_id = self._db.add_snapshot(
            room_label=msg.room_label,
            room_confidence=msg.room_confidence,
            pose_x=msg.robot_pose.position.x,
            pose_y=msg.robot_pose.position.y,
            objects=objects,
        )
        self.get_logger().info(
            f'Graph updated: {room_id} | nodes={self._db.graph.number_of_nodes()}'
        )
        self._publish_markers()

    def _query_cb(self, request: SemanticQuery.Request, response: SemanticQuery.Response):
        result = self._db.query_location(request.query_text)
        response.known_locations = self._db.known_locations()

        if result is None:
            response.success = False
            response.message = (
                f"Location not found for: '{request.query_text}'. "
                f"Known locations: {response.known_locations}"
            )
            return response

        x, y, label, conf = result
        response.success = True
        response.matched_label = label
        response.confidence = conf
        response.message = f"Found '{label}' (confidence={conf:.2f})"

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        response.goal_pose = ps

        self.get_logger().info(response.message)
        return response

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0

        for node_id, data in self._db.graph.nodes(data=True):
            if data.get('type') != 'room':
                continue

            # Room label text
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'room_labels'
            m.id = mid; mid += 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = data['pose_x']
            m.pose.position.y = data['pose_y']
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.z = 0.4
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            m.text = f"{data['label']}\n({data['confidence']:.2f})"
            m.lifetime = Duration(sec=10)
            ma.markers.append(m)

            # Room footprint circle
            m2 = Marker()
            m2.header.frame_id = 'map'
            m2.header.stamp = now
            m2.ns = 'room_areas'
            m2.id = mid; mid += 1
            m2.type = Marker.CYLINDER
            m2.action = Marker.ADD
            m2.pose.position.x = data['pose_x']
            m2.pose.position.y = data['pose_y']
            m2.pose.position.z = 0.02
            m2.pose.orientation.w = 1.0
            m2.scale.x = 2.0; m2.scale.y = 2.0; m2.scale.z = 0.05
            m2.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.2)
            m2.lifetime = Duration(sec=10)
            ma.markers.append(m2)

        self._marker_pub.publish(ma)

    def _save_and_exit(self):
        self.get_logger().info('Saving semantic graph before exit...')
        self._db.save()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GraphServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
