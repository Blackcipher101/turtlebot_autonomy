"""Nav2 NavigateToPose action client with retry on failure."""

from __future__ import annotations
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus


class Nav2Client:
    def __init__(self, node: Node):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._status_pub = node.create_publisher(String, '/semantic_nav_status', 10)

    def navigate_to(self, pose: PoseStamped, retries: int = 1) -> bool:
        """Send Nav2 goal, wait for result. Returns True on success."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error('Nav2 action server not available')
            return False

        for attempt in range(retries + 1):
            self._publish_status(f'NAVIGATING to ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})')
            goal = NavigateToPose.Goal()
            goal.pose = pose

            future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

            if not future.result() or not future.result().accepted:
                self._node.get_logger().warn('Nav2 goal rejected')
                continue

            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=120.0)

            status = result_future.result().status if result_future.result() else GoalStatus.STATUS_UNKNOWN
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._publish_status('ARRIVED')
                self._node.get_logger().info('Navigation succeeded.')
                return True

            self._node.get_logger().warn(f'Nav2 failed (status={status}), attempt {attempt+1}')
            if attempt < retries:
                self._publish_status('RETRYING')
                time.sleep(1.0)

        self._publish_status('FAILED')
        return False

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self._node.get_logger().info(f'[SemanticNav] {text}')
