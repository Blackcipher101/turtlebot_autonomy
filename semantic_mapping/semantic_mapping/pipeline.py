"""
Main pipeline:
  bag → frames → SAM (segment everything) → CLIP embed each crop
  → BLIP-2 caption each crop → 3D position from LiDAR poses
  → semantic graph with spatial + semantic edges
"""
import os
import cv2
import numpy as np
from pathlib import Path
from typing import Optional

from .config import Config
from .bag_reader import BagReader
from .image_renderer import make_detection_image
from .segment_everything import SAMEverything
from .clip_embedder import CLIPEmbedder
from .scene_descriptor import SceneDescriptor, CLIPSceneClassifier
from .position_estimator import BEVPositionEstimator, load_poses_tum, nearest_pose
from .semantic_graph import SemanticGraph, SemanticNode

BEV_X = (-30.0, 30.0)
BEV_Y = (-30.0, 30.0)
BEV_W, BEV_H = 600, 600


class SemanticMappingPipeline:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        Path(cfg.output_dir).mkdir(parents=True, exist_ok=True)
        Path(cfg.output_dir, "crops").mkdir(exist_ok=True)

        print("=" * 60)
        print("  Semantic Mapping Pipeline  (SAM → CLIP → Graph)")
        print("=" * 60)

        self.sam   = SAMEverything(cfg.sam_checkpoint, cfg.sam_model_type)
        self.clip  = CLIPEmbedder(cfg.clip_model)
        self.scene_clf = CLIPSceneClassifier(self.clip)

        # BLIP-2 is optional (heavy) — only load if flag set
        self.blip = None
        if cfg.use_blip:
            self.blip = SceneDescriptor()

        self.graph = SemanticGraph(cfg.spatial_edge_dist)

        # Poses
        self.poses = {}
        if cfg.poses_file and Path(cfg.poses_file).exists():
            self.poses = load_poses_tum(cfg.poses_file)
            print(f"[Pipeline] {len(self.poses)} poses loaded")
        else:
            print("[Pipeline] No poses — using identity (robot stays at origin)")

        self.bag = BagReader(cfg.bag_path, cfg.lidar_topic, cfg.camera_topic)

    def run(self):
        topics = self.bag.list_topics()
        has_camera = bool(self.cfg.camera_topic) and self.cfg.camera_topic in topics
        print(f"[Pipeline] Topics in bag: {topics}")
        print(f"[Pipeline] Mode: {'camera' if has_camera else 'LiDAR-render'}")

        if has_camera:
            self._run_camera()
        else:
            self._run_lidar_render()

        print("[Pipeline] Merging duplicate segments...")
        self.graph.merge_duplicates()

        stats = self.graph.stats()
        print(f"\n[Pipeline] Final graph: {stats}")

        self.graph.save(self.cfg.graph_file)
        self.graph.visualize_2d(
            save_path=str(Path(self.cfg.output_dir) / "semantic_map.png")
        )
        print("[Pipeline] Done ✓")
        return self.graph

    def _process_frame(self, timestamp: float, image_bgr: np.ndarray,
                        pts: Optional[np.ndarray], frame_idx: int):
        h, w = image_bgr.shape[:2]
        pose = nearest_pose(self.poses, timestamp) if self.poses else np.eye(4)

        # Scene type for this frame
        scene_type, scene_conf = self.scene_clf.classify(image_bgr)

        # SAM — segment everything
        segments = self.sam.segment(image_bgr)
        if not segments:
            return

        # CLIP embed all crops at once (batch)
        crops = [s.crop for s in segments]
        try:
            clip_embs = self.clip.embed_images(crops)   # (N, D)
        except Exception as e:
            print(f"[Pipeline] CLIP batch failed: {e}")
            return

        added = 0
        for i, (seg, emb) in enumerate(zip(segments, clip_embs)):
            # Optional BLIP caption
            caption = ""
            if self.blip:
                try:
                    caption = self.blip.describe_crop(seg.crop)
                except Exception:
                    pass

            # 3D position from BEV pixel
            if pts is not None:
                x_px, y_px, bw, bh = seg.bbox
                cx_n = (x_px + bw / 2) / w
                cy_n = (y_px + bh / 2) / h
                estimator = BEVPositionEstimator(pose, BEV_X, BEV_Y, BEV_W, BEV_H)
                position = estimator.pixel_to_world(cx_n, cy_n, pts)
            else:
                position = pose[:3, 3].copy()

            # Save crop
            crop_path = str(Path(self.cfg.output_dir) / "crops" /
                            f"f{frame_idx:04d}_s{i:03d}.jpg")
            cv2.imwrite(crop_path, seg.crop)

            node = SemanticNode(
                id=0,
                position=position,
                clip_image_emb=emb,
                scene_type=scene_type,
                blip_caption=caption,
                score=(seg.stability + seg.iou) / 2,
                frame_id=frame_idx,
                timestamp=timestamp,
                bbox=seg.bbox,
                area=seg.area,
            )
            self.graph.add_node(node)
            added += 1

        print(f"[Pipeline] Frame {frame_idx:4d} | t={timestamp:.1f}s | "
              f"scene={scene_type}({scene_conf:.2f}) | {added} segments added")

        # Save annotated frame
        vis = image_bgr.copy()
        for seg in segments:
            x, y, bw, bh = seg.bbox
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 200, 0), 1)
        cv2.imwrite(str(Path(self.cfg.output_dir) / f"frame_{frame_idx:05d}.jpg"), vis)

    def _run_lidar_render(self):
        frame_idx = 0
        for timestamp, pts in self.bag.lidar_frames(
            self.cfg.lidar_every_n, self.cfg.max_frames
        ):
            image = make_detection_image(pts)
            self._process_frame(timestamp, image, pts, frame_idx)
            frame_idx += 1

    def _run_camera(self):
        # Build LiDAR timestamp lookup for depth
        lidar_buf = {}
        for ts, pts in self.bag.lidar_frames(1, self.cfg.max_frames * 5):
            lidar_buf[ts] = pts
        lidar_ts = sorted(lidar_buf.keys())

        frame_idx = 0
        for timestamp, image_bgr in self.bag.camera_frames(
            self.cfg.lidar_every_n, self.cfg.max_frames
        ):
            pts = None
            if lidar_ts:
                nearest_t = lidar_ts[int(np.argmin(
                    np.abs(np.array(lidar_ts) - timestamp)
                ))]
                pts = lidar_buf.get(nearest_t)
            self._process_frame(timestamp, image_bgr, pts, frame_idx)
            frame_idx += 1


def query_graph(graph_path: str, query_text: str,
                clip_model: str = "ViT-B/32") -> Optional[np.ndarray]:
    """Load saved graph and return 3D position for a text query."""
    graph = SemanticGraph.load(graph_path)
    clip  = CLIPEmbedder(clip_model)
    return graph.get_navigation_target(query_text, clip)
