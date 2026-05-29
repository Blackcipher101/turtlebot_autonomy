"""SAM Automatic Mask Generator — segments everything in the image."""
import numpy as np
import cv2
import torch
from typing import List
from dataclasses import dataclass


@dataclass
class Segment:
    mask: np.ndarray        # HxW bool
    bbox: tuple             # (x, y, w, h) pixel
    crop: np.ndarray        # HxWx3 BGR cropped + masked
    area: int
    stability: float
    iou: float


def _crop_with_mask(image_bgr: np.ndarray, mask: np.ndarray,
                    bbox: tuple, padding: int = 4) -> np.ndarray:
    x, y, w, h = bbox
    H, W = image_bgr.shape[:2]
    x1, y1 = max(0, x - padding), max(0, y - padding)
    x2, y2 = min(W, x + w + padding), min(H, y + h + padding)
    crop = image_bgr[y1:y2, x1:x2].copy()
    crop[~mask[y1:y2, x1:x2]] = 0
    return crop


class SAMEverything:
    def __init__(self, checkpoint: str, model_type: str = "vit_h",
                 device: str = "cuda",
                 min_area_fraction: float = 0.001,
                 max_area_fraction: float = 0.5,
                 points_per_side: int = 32,
                 iou_threshold: float = 0.80,
                 stability_threshold: float = 0.90):
        self.device = device if torch.cuda.is_available() else "cpu"
        self.min_area_fraction = min_area_fraction
        self.max_area_fraction = max_area_fraction

        print(f"[SAM] Loading {model_type} on {self.device}...")
        from segment_anything import sam_model_registry, SamAutomaticMaskGenerator
        sam = sam_model_registry[model_type](checkpoint=checkpoint).to(self.device)
        self.generator = SamAutomaticMaskGenerator(
            model=sam,
            points_per_side=points_per_side,
            pred_iou_thresh=iou_threshold,
            stability_score_thresh=stability_threshold,
            min_mask_region_area=100,
        )
        print("[SAM] Ready")

    def segment(self, image_bgr: np.ndarray) -> List[Segment]:
        H, W = image_bgr.shape[:2]
        total = H * W
        raw = self.generator.generate(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
        out = []
        for m in raw:
            frac = m["area"] / total
            if not (self.min_area_fraction <= frac <= self.max_area_fraction):
                continue
            mask = m["segmentation"].astype(bool)
            bbox = tuple(int(v) for v in m["bbox"])
            crop = _crop_with_mask(image_bgr, mask, bbox)
            if crop.shape[0] < 4 or crop.shape[1] < 4:
                continue
            out.append(Segment(
                mask=mask, bbox=bbox, crop=crop,
                area=int(m["area"]),
                stability=float(m.get("stability_score", 1.0)),
                iou=float(m.get("predicted_iou", 1.0)),
            ))
        out.sort(key=lambda s: -s.area)
        return out
