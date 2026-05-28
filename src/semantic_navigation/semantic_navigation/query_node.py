"""Semantic query node: text → graph lookup → Nav2 goal.

CLI usage (from another terminal inside the container):
  ros2 service call /semantic_query semantic_msgs/srv/SemanticQuery \
    "{query_text: 'Go to the kitchen'}"
"""

import rclpy
from rclpy.node import Node

from semantic_msgs.srv import SemanticQuery
from .nav_client import Nav2Client


class QueryNode(Node):
    def __init__(self):
        super().__init__('semantic_query_node')

        # Client to the graph server's query service
        self._graph_client = self.create_client(SemanticQuery, '/semantic_query')

        # Nav2 wrapper
        self._nav = Nav2Client(self)

        # Expose a top-level /navigate_semantic service for users
        self.create_service(SemanticQuery, '/navigate_semantic', self._navigate_cb)

        self.get_logger().info(
            'SemanticQueryNode ready. '
            'Use: ros2 service call /navigate_semantic semantic_msgs/srv/SemanticQuery '
            '"{query_text: \'Go to the kitchen\'}"'
        )

    def _navigate_cb(
        self,
        request: SemanticQuery.Request,
        response: SemanticQuery.Response,
    ) -> SemanticQuery.Response:
        query = request.query_text
        self.get_logger().info(f"Received semantic query: '{query}'")

        # 1. Look up location in semantic graph
        if not self._graph_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = 'Semantic graph server not available'
            return response

        graph_req = SemanticQuery.Request()
        graph_req.query_text = query
        future = self._graph_client.call_async(graph_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.result():
            response.success = False
            response.message = 'Graph query timed out'
            return response

        graph_resp = future.result()
        response.known_locations = graph_resp.known_locations

        if not graph_resp.success:
            response.success = False
            response.message = graph_resp.message
            return response

        # 2. Send Nav2 goal
        self.get_logger().info(
            f"Navigating to '{graph_resp.matched_label}' at "
            f"({graph_resp.goal_pose.pose.position.x:.2f}, "
            f"{graph_resp.goal_pose.pose.position.y:.2f})"
        )
        success = self._nav.navigate_to(graph_resp.goal_pose, retries=1)

        response.success = success
        response.matched_label = graph_resp.matched_label
        response.confidence = graph_resp.confidence
        response.goal_pose = graph_resp.goal_pose
        response.message = 'Navigation succeeded' if success else 'Navigation failed'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = QueryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
