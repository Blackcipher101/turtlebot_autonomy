"""
Back-project detected objects to 3D world positions using LiDAR poses.

For BEV images: pixel → world XY is a direct grid lookup.
For camera images: use camera intrinsics + depth from nearest LiDAR point.
"""
import numpy as np
from typing import Dict, Any, Optional, Tuple


class BEVPositionEstimator:
    """Maps pixel positions in BEV image back to world frame."""
    def __init__(self,
                 pose: np.ndarray,            # 4x4 SE3 world←body transform
                 x_range: Tuple[float,float] = (-30, 30),
                 y_range: Tuple[float,float] = (-30, 30),
                 img_w: int = 600, img_h: int = 600):
        self.pose    = pose
        self.x_range = x_range
        self.y_range = y_range
        self.img_w   = img_w
        self.img_h   = img_h

    def pixel_to_world(self, cx_norm: float, cy_norm: float,
                        pts_body: np.ndarray) -> np.ndarray:
        """
        cx_norm, cy_norm: normalized [0,1] center of detection in BEV image.
        pts_body: LiDAR points in body frame (Nx4).
        Returns 3D world position [x, y, z].
        """
        # BEV image: cx_norm=0 → x=x_range[0], cx_norm=1 → x=x_range[1]
        local_x = cx_norm * (self.x_range[1] - self.x_range[0]) + self.x_range[0]
        local_y = cy_norm * (self.y_range[1] - self.y_range[0]) + self.y_range[0]

        # Find nearest LiDAR point to get z (height)
        dists = np.sqrt((pts_body[:, 0] - local_x)**2 + (pts_body[:, 1] - local_y)**2)
        if len(dists) > 0:
            local_z = float(pts_body[np.argmin(dists), 2])
        else:
            local_z = 0.0

        local_pt = np.array([local_x, local_y, local_z, 1.0])
        world_pt = self.pose @ local_pt
        return world_pt[:3]


class CameraPositionEstimator:
    """Back-project camera detections to 3D using LiDAR depth."""
    def __init__(self, pose: np.ndarray,
                 K: np.ndarray,         # 3x3 camera intrinsics
                 lidar_to_cam: np.ndarray):  # 4x4 extrinsic
        self.pose         = pose
        self.K            = K
        self.lidar_to_cam = lidar_to_cam

    def pixel_to_world(self, cx_norm: float, cy_norm: float,
                        pts_body: np.ndarray, img_w: int, img_h: int) -> np.ndarray:
        """
        Use camera ray + LiDAR depth to get 3D world position.
        """
        # Project LiDAR to camera frame
        pts_h = np.hstack([pts_body[:, :3], np.ones((len(pts_body), 1))])
        pts_cam = (self.lidar_to_cam @ pts_h.T).T[:, :3]
        # Keep only points in front
        front = pts_cam[pts_cam[:, 2] > 0.1]
        if len(front) == 0:
            return np.array([0., 0., 0.])

        # Project to image
        uvw = (self.K @ front.T).T
        u = uvw[:, 0] / uvw[:, 2]
        v = uvw[:, 1] / uvw[:, 2]

        cx_px = cx_norm * img_w
        cy_px = cy_norm * img_h
        dists = np.sqrt((u - cx_px)**2 + (v - cy_px)**2)
        nearest = front[np.argmin(dists)]  # [x,y,z] in camera frame

        # Camera → body → world
        cam_pt = np.array([*nearest, 1.0])
        body_pt = np.linalg.inv(self.lidar_to_cam) @ cam_pt
        world_pt = self.pose @ body_pt
        return world_pt[:3]


def load_poses_tum(poses_file: str) -> Dict[float, np.ndarray]:
    """
    Load TUM-format poses: timestamp tx ty tz qx qy qz qw
    Returns dict {timestamp: 4x4 SE3}.
    """
    from scipy.spatial.transform import Rotation
    poses = {}
    with open(poses_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            poses[t] = T
    return poses


def nearest_pose(poses: Dict[float, np.ndarray], timestamp: float) -> np.ndarray:
    """Return closest pose by timestamp."""
    if not poses:
        return np.eye(4)
    ts = np.array(list(poses.keys()))
    idx = np.argmin(np.abs(ts - timestamp))
    return poses[ts[idx]]
