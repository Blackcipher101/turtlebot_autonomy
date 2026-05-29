#!/usr/bin/env python3
"""
ROS2 node: listen for text queries on /semantic_query,
           search the graph, send Nav2 goal to matched position.

Usage:
  ros2 run (or python) nav2_query_node.py --graph /mnt/data/semantic_output/semantic_graph.pkl

  # Send a query:
  ros2 topic pub /semantic_query std_msgs/msg/String "data: 'red bus in garden'"
"""
import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from semantic_mapping import SemanticGraph
from semantic_mapping.clip_embedder import CLIPEmbedder


class SemanticQueryNode(Node):
    def __init__(self, graph_path: str, clip_model: str = "ViT-B/32"):
        super().__init__("semantic_query_node")
        self.get_logger().info(f"Loading graph from {graph_path}...")
        self.graph = SemanticGraph.load(graph_path)
        self.clip  = CLIPEmbedder(clip_model)
        self.get_logger().info(
            f"Graph loaded: {self.graph.stats()['nodes']} objects"
        )

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._query_sub  = self.create_subscription(
            String, "/semantic_query", self._on_query, 10
        )
        self.get_logger().info("Listening for queries on /semantic_query")

    def _on_query(self, msg: String):
        query = msg.data.strip()
        self.get_logger().info(f"Query received: '{query}'")

        position = self.graph.get_navigation_target(query, self.clip)
        if position is None:
            self.get_logger().warn(f"No match found for '{query}'")
            return

        self.get_logger().info(
            f"Navigating to {position} for query '{query}'"
        )
        self._send_nav_goal(position)

    def _send_nav_goal(self, position: np.ndarray):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 action server not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position[0])
        goal.pose.pose.position.y = float(position[1])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending Nav2 goal: ({position[0]:.2f}, {position[1]:.2f})"
        )
        self._nav_client.send_goal_async(goal)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--graph", default="/mnt/data/semantic_output/semantic_graph.pkl")
    p.add_argument("--clip-model", default="ViT-B/32")
    args = p.parse_args()

    rclpy.init()
    node = SemanticQueryNode(args.graph, args.clip_model)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
