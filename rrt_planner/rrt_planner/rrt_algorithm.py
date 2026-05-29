"""Pure RRT* algorithm — zero ROS dependencies.

All coordinates are in world space (metres). The occupancy grid is a 2D
numpy array (row=y, col=x) with values: -1=unknown, 0=free, 1-100=occupied.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import binary_dilation

XY = Tuple[float, float]


@dataclass
class RRTNode:
    x: float
    y: float
    parent: Optional[int] = None  # index into node list
    cost: float = 0.0             # cumulative cost from root


class RRTStar:
    """RRT* planner with goal biasing, rewiring, and path smoothing."""

    def __init__(
        self,
        max_iterations: int = 5000,
        step_size: float = 0.10,
        goal_tolerance: float = 0.20,
        goal_bias: float = 0.15,
        rewire_radius: float = 0.50,
        inflate_radius_m: float = 0.12,
        allow_unknown: bool = False,
    ):
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_tolerance = goal_tolerance
        self.goal_bias = goal_bias
        self.rewire_radius = rewire_radius
        self.inflate_radius_m = inflate_radius_m
        self.allow_unknown = allow_unknown

    def plan(
        self,
        start: XY,
        goal: XY,
        raw_grid: np.ndarray,   # shape (H, W), dtype int16
        resolution: float,
        origin: XY,
    ) -> Tuple[List[XY], List[XY], List[Tuple[XY, XY]]]:
        """
        Returns:
          path        — smoothed list of (x,y) world coords, or []
          node_xys    — all tree node positions (for visualization)
          edge_pairs  — (parent_xy, child_xy) pairs (for visualization)
        """
        grid = self._inflate(raw_grid, resolution)
        H, W = grid.shape

        # Validate start/goal
        if not self._free(start, grid, resolution, origin):
            return [], [], []

        x0, y0 = origin
        nodes: List[RRTNode] = [RRTNode(start[0], start[1])]
        best_goal_idx: Optional[int] = None

        for _ in range(self.max_iterations):
            # Sample — with goal bias
            if random.random() < self.goal_bias:
                q = goal
            else:
                for _ in range(50):
                    rx = random.uniform(x0, x0 + W * resolution)
                    ry = random.uniform(y0, y0 + H * resolution)
                    if self._free((rx, ry), grid, resolution, origin):
                        q = (rx, ry)
                        break
                else:
                    continue

            near_idx = self._nearest(nodes, q)
            q_new = self._steer((nodes[near_idx].x, nodes[near_idx].y), q)

            if not self._free(q_new, grid, resolution, origin):
                continue
            if not self._los(
                (nodes[near_idx].x, nodes[near_idx].y), q_new,
                grid, resolution, origin
            ):
                continue

            # Choose best parent (RRT*)
            nbrs = self._near(nodes, q_new)
            best_p, best_c = near_idx, nodes[near_idx].cost + self._d(
                (nodes[near_idx].x, nodes[near_idx].y), q_new
            )
            for i in nbrs:
                c = nodes[i].cost + self._d((nodes[i].x, nodes[i].y), q_new)
                if c < best_c and self._los(
                    (nodes[i].x, nodes[i].y), q_new, grid, resolution, origin
                ):
                    best_p, best_c = i, c

            nn = RRTNode(q_new[0], q_new[1], parent=best_p, cost=best_c)
            new_idx = len(nodes)
            nodes.append(nn)

            # Rewire (RRT*)
            for i in nbrs:
                c = best_c + self._d(q_new, (nodes[i].x, nodes[i].y))
                if c < nodes[i].cost and self._los(
                    q_new, (nodes[i].x, nodes[i].y), grid, resolution, origin
                ):
                    nodes[i].parent = new_idx
                    nodes[i].cost = c

            # Goal check
            if self._d(q_new, goal) <= self.goal_tolerance:
                if best_goal_idx is None or best_c < nodes[best_goal_idx].cost:
                    best_goal_idx = new_idx

        # Extract and smooth path
        if best_goal_idx is None:
            path: List[XY] = []
        else:
            path = self._extract(nodes, best_goal_idx)
            path = self._smooth(path, grid, resolution, origin)

        node_xys = [(n.x, n.y) for n in nodes]
        edges = [
            ((nodes[n.parent].x, nodes[n.parent].y), (n.x, n.y))
            for n in nodes if n.parent is not None
        ]
        return path, node_xys, edges

    # ── Grid helpers ─────────────────────────────────────────────────

    def _inflate(self, raw: np.ndarray, res: float) -> np.ndarray:
        r = max(1, int(math.ceil(self.inflate_radius_m / res)))
        struct = np.ones((2 * r + 1, 2 * r + 1), dtype=bool)
        if self.allow_unknown:
            occ = raw >= 65
        else:
            occ = (raw >= 65) | (raw < 0)
        return binary_dilation(occ, structure=struct)

    def _rc(
        self, xy: XY, res: float, origin: XY, shape: Tuple[int, int]
    ) -> Optional[Tuple[int, int]]:
        col = int((xy[0] - origin[0]) / res)
        row = int((xy[1] - origin[1]) / res)
        H, W = shape
        if 0 <= row < H and 0 <= col < W:
            return row, col
        return None

    def _free(self, xy: XY, grid: np.ndarray, res: float, origin: XY) -> bool:
        rc = self._rc(xy, res, origin, grid.shape)
        return rc is not None and not grid[rc[0], rc[1]]

    def _los(
        self, p1: XY, p2: XY, grid: np.ndarray, res: float, origin: XY
    ) -> bool:
        """Line-of-sight collision check at sub-cell resolution."""
        d = self._d(p1, p2)
        if d < 1e-9:
            return True
        steps = max(2, int(d / (res * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self._free((x, y), grid, res, origin):
                return False
        return True

    # ── Tree operations ──────────────────────────────────────────────

    @staticmethod
    def _d(a: XY, b: XY) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def _nearest(self, nodes: List[RRTNode], q: XY) -> int:
        xs = np.array([n.x for n in nodes])
        ys = np.array([n.y for n in nodes])
        return int(np.argmin(np.hypot(xs - q[0], ys - q[1])))

    def _near(self, nodes: List[RRTNode], q: XY) -> List[int]:
        xs = np.array([n.x for n in nodes])
        ys = np.array([n.y for n in nodes])
        return list(np.where(np.hypot(xs - q[0], ys - q[1]) <= self.rewire_radius)[0])

    def _steer(self, src: XY, dst: XY) -> XY:
        d = self._d(src, dst)
        if d <= self.step_size:
            return dst
        r = self.step_size / d
        return src[0] + r * (dst[0] - src[0]), src[1] + r * (dst[1] - src[1])

    # ── Path helpers ─────────────────────────────────────────────────

    @staticmethod
    def _extract(nodes: List[RRTNode], idx: int) -> List[XY]:
        path: List[XY] = []
        i: Optional[int] = idx
        while i is not None:
            path.append((nodes[i].x, nodes[i].y))
            i = nodes[i].parent
        path.reverse()
        return path

    def _smooth(
        self, path: List[XY], grid: np.ndarray, res: float, origin: XY
    ) -> List[XY]:
        """Greedy line-of-sight shortcutting: O(n) passes."""
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._los(path[i], path[j], grid, res, origin):
                    break
                j -= 1
            out.append(path[j])
            i = j
        return out
