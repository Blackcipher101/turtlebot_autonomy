#!/usr/bin/env python3
"""RRT* Planner + Robot Path Follower.

Flow:
  1. Gazebo spawns burger, SLAM builds map, Nav2 activates
  2. User sets "2D Pose Estimate" → start pose
  3. User sets "2D Goal Pose"     → triggers RRT* planning
  4. RRT* computes path, publishes to /rrt_path + /rrt_tree
  5. Path sent to Nav2 /follow_path action → robot physically drives
"""

import threading
from typing import Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray

from rrt_planner.rrt_algorithm import RRTStar

XY = Tuple[float, float]


class RRTPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('rrt_planner_node')

        # Parameters
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('step_size', 0.10)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('goal_bias', 0.15)
        self.declare_parameter('rewire_radius', 0.50)
        self.declare_parameter('inflate_radius_m', 0.12)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('auto_follow', True)

        p = lambda n: self.get_parameter(n).value
        self._rrt = RRTStar(
            max_iterations=p('max_iterations'), step_size=p('step_size'),
            goal_tolerance=p('goal_tolerance'), goal_bias=p('goal_bias'),
            rewire_radius=p('rewire_radius'), inflate_radius_m=p('inflate_radius_m'),
            allow_unknown=p('allow_unknown'),
        )
        self._auto_follow = p('auto_follow')

        # State
        self._map: Optional[OccupancyGrid] = None
        self._start: Optional[XY] = None
        self._lock = threading.Lock()

        # ReentrantCallbackGroup allows callbacks to run concurrently
        # (needed so action futures don't deadlock the executor)
        self._cb_group = ReentrantCallbackGroup()

        # QoS
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        )

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos,
                                 callback_group=self._cb_group)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self._start_cb, 10, callback_group=self._cb_group)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, 10, callback_group=self._cb_group)
        self.create_subscription(PoseStamped, '/goal_pose',
                                 self._goal_cb, 10, callback_group=self._cb_group)

        # Publishers
        self._path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self._tree_pub = self.create_publisher(MarkerArray, '/rrt_tree', 10)

        # FollowPath action client
        self._follow_client = ActionClient(
            self, FollowPath, '/follow_path',
            callback_group=self._cb_group
        )
        self._goal_handle = None

        self.get_logger().info(
            f'RRT* planner ready  (auto_follow={self._auto_follow})\n'
            '  [RViz] "2D Pose Estimate" → set start\n'
            '  [RViz] "2D Goal Pose"     → plan + follow'
        )

    # Callbacks

    def _map_cb(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map = msg
        self.get_logger().info(
            f'Map: {msg.info.width}×{msg.info.height} '
            f'@ {msg.info.resolution:.3f}m/cell  '
            f'origin=({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})'
        )

    def _start_cb(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f'Start: {self._start}')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _goal_cb(self, msg: PoseStamped) -> None:
        # Cancel any active follow goal
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        with self._lock:
            map_snap = self._map
            start_snap = self._start

        if map_snap is None:
            self.get_logger().warn('No map yet.')
            return

        if start_snap is None:
            start_snap = (
                map_snap.info.origin.position.x + 2 * map_snap.info.resolution,
                map_snap.info.origin.position.y + 2 * map_snap.info.resolution,
            )
            self.get_logger().warn('No /initialpose — using map origin fallback.')

        goal: XY = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Planning RRT*: {start_snap} → {goal}')

        raw = np.array(map_snap.data, dtype=np.int16).reshape(
            map_snap.info.height, map_snap.info.width)
        origin: XY = (map_snap.info.origin.position.x, map_snap.info.origin.position.y)
        res = map_snap.info.resolution
        frame = map_snap.header.frame_id or 'map'

        path, node_xys, edges = self._rrt.plan(start_snap, goal, raw, res, origin)

        if path:
            self.get_logger().info(
                f'Path found: {len(path)} waypoints ({len(node_xys)} tree nodes)')
        else:
            self.get_logger().warn('RRT* failed — no path found.')
            self._path_pub.publish(self._make_path([], frame))
            self._tree_pub.publish(self._make_tree_markers(node_xys, edges, frame))
            return

        path_msg = self._make_path(path, frame)
        self._path_pub.publish(path_msg)
        self._tree_pub.publish(self._make_tree_markers(node_xys, edges, frame))

        if self._auto_follow:
            self._follow(path_msg)

    def _follow(self, path_msg: Path) -> None:
        if not self._follow_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                '/follow_path server unavailable — is Nav2 running? '
                'Path published to /rrt_path only.'
            )
            return

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = ''
        goal.goal_checker_id = ''

        self.get_logger().info('Sending path to Nav2 controller → robot moving...')
        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('FollowPath goal REJECTED.')
            return
        self._goal_handle = handle
        self.get_logger().info('FollowPath accepted — robot following RRT* path!')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self._goal_handle = None
        status = future.result().status
        msgs = {4: 'Robot reached goal!', 5: 'Path following cancelled.'}
        msg = msgs.get(status, f'Follow ended (status={status}).')
        self.get_logger().info(msg)

    # Message builders

    def _make_path(self, path, frame: str) -> Path:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _make_tree_markers(self, nodes, edges, frame: str) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clr = Marker(); clr.header.stamp = now; clr.header.frame_id = frame
        clr.action = Marker.DELETEALL; ma.markers.append(clr)

        em = Marker()
        em.header.stamp = now; em.header.frame_id = frame
        em.ns = 'rrt_edges'; em.id = 1; em.type = Marker.LINE_LIST
        em.action = Marker.ADD; em.scale.x = 0.015
        em.color.r = 0.1; em.color.g = 0.6; em.color.b = 1.0; em.color.a = 0.5
        em.pose.orientation.w = 1.0; em.lifetime = Duration(sec=0, nanosec=0)
        for (x1, y1), (x2, y2) in edges:
            p1 = Point(); p1.x = x1; p1.y = y1; p1.z = 0.02
            p2 = Point(); p2.x = x2; p2.y = y2; p2.z = 0.02
            em.points.extend([p1, p2])
        ma.markers.append(em)

        nm = Marker()
        nm.header.stamp = now; nm.header.frame_id = frame
        nm.ns = 'rrt_nodes'; nm.id = 2; nm.type = Marker.SPHERE_LIST
        nm.action = Marker.ADD
        nm.scale.x = nm.scale.y = nm.scale.z = 0.04
        nm.color.r = 1.0; nm.color.g = 0.5; nm.color.b = 0.0; nm.color.a = 0.8
        nm.pose.orientation.w = 1.0; nm.lifetime = Duration(sec=0, nanosec=0)
        for x, y in nodes:
            p = Point(); p.x = x; p.y = y; p.z = 0.02
            nm.points.append(p)
        ma.markers.append(nm)
        return ma


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
