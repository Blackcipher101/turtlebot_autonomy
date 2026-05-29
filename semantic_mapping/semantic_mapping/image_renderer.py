"""Render LiDAR point clouds into images when no camera is available."""
import numpy as np
import cv2
from typing import Tuple


def render_bev(pts: np.ndarray, resolution: float = 0.1,
               x_range: Tuple[float, float] = (-30, 30),
               y_range: Tuple[float, float] = (-30, 30)) -> np.ndarray:
    """
    Bird's eye view (top-down) intensity image from LiDAR Nx4 (x,y,z,intensity).
    Returns HxWx3 BGR image.
    """
    W = int((x_range[1] - x_range[0]) / resolution)
    H = int((y_range[1] - y_range[0]) / resolution)
    canvas = np.zeros((H, W), dtype=np.float32)
    count  = np.zeros((H, W), dtype=np.float32)

    mask = (
        (pts[:, 0] >= x_range[0]) & (pts[:, 0] < x_range[1]) &
        (pts[:, 1] >= y_range[0]) & (pts[:, 1] < y_range[1])
    )
    p = pts[mask]
    if len(p) == 0:
        return np.zeros((H, W, 3), dtype=np.uint8)

    col = ((p[:, 0] - x_range[0]) / resolution).astype(int)
    row = ((p[:, 1] - y_range[0]) / resolution).astype(int)
    col = np.clip(col, 0, W - 1)
    row = np.clip(row, 0, H - 1)

    intensity = p[:, 3] if p.shape[1] > 3 else np.ones(len(p))
    np.add.at(canvas, (row, col), intensity)
    np.add.at(count,  (row, col), 1)
    valid = count > 0
    canvas[valid] /= count[valid]

    img = (canvas / (canvas.max() + 1e-6) * 255).astype(np.uint8)
    return cv2.applyColorMap(img, cv2.COLORMAP_INFERNO)


def render_front_projection(pts: np.ndarray,
                             h_fov: Tuple[float, float] = (-np.pi, np.pi),
                             v_fov: Tuple[float, float] = (-0.4, 0.4),
                             w: int = 1024, h: int = 128) -> np.ndarray:
    """
    Spherical / cylindrical front-facing projection (like a camera panorama).
    Returns HxWx3 BGR image using intensity + height coloring.
    """
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.sqrt(x**2 + y**2 + z**2) + 1e-9

    azimuth  = np.arctan2(y, x)
    elevation = np.arcsin(np.clip(z / r, -1, 1))

    az_mask = (azimuth >= h_fov[0]) & (azimuth <= h_fov[1])
    el_mask = (elevation >= v_fov[0]) & (elevation <= v_fov[1])
    mask = az_mask & el_mask
    pts_m = pts[mask]
    az_m  = azimuth[mask]
    el_m  = elevation[mask]

    if len(pts_m) == 0:
        return np.zeros((h, w, 3), dtype=np.uint8)

    col = ((az_m - h_fov[0]) / (h_fov[1] - h_fov[0]) * (w - 1)).astype(int)
    row = ((1 - (el_m - v_fov[0]) / (v_fov[1] - v_fov[0])) * (h - 1)).astype(int)
    col = np.clip(col, 0, w - 1)
    row = np.clip(row, 0, h - 1)

    canvas = np.zeros((h, w), dtype=np.float32)
    intensity = pts_m[:, 3] if pts_m.shape[1] > 3 else np.ones(len(pts_m))
    np.maximum.at(canvas, (row, col), intensity)

    img = (canvas / (canvas.max() + 1e-6) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    # Upscale for better detection
    return cv2.resize(colored, (w, h * 4), interpolation=cv2.INTER_NEAREST)


def make_detection_image(pts: np.ndarray) -> np.ndarray:
    """
    Combine BEV + front projection vertically for detection.
    Returns a single image fed to Grounding DINO.
    """
    bev   = render_bev(pts)
    front = render_front_projection(pts)
    # Resize to same width
    w = max(bev.shape[1], front.shape[1])
    bev   = cv2.resize(bev,   (w, int(bev.shape[0]   * w / bev.shape[1])))
    front = cv2.resize(front, (w, int(front.shape[0] * w / front.shape[1])))
    return np.vstack([bev, front])
