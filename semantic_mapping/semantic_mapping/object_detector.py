"""Grounding DINO + SAM object detection and segmentation."""
import numpy as np
import torch
import cv2
from typing import List, Dict, Any, Optional
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from PIL import Image


class GroundingDINO:
    def __init__(self, model_id: str = "IDEA-Research/grounding-dino-base",
                 threshold: float = 0.35, device: str = "cuda"):
        self.device = device if torch.cuda.is_available() else "cpu"
        print(f"[GroundingDINO] Loading {model_id} on {self.device}...")
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(self.device)
        self.threshold = threshold
        print("[GroundingDINO] Ready")

    def detect(self, image_bgr: np.ndarray, text_query: str) -> List[Dict[str, Any]]:
        """
        Returns list of dicts: {label, score, box_xyxy, box_cx_cy_norm}
        text_query: "car . person . tree . building ." (dot-separated)
        """
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(image_rgb)
        h, w = image_bgr.shape[:2]

        inputs = self.processor(
            images=pil_image,
            text=text_query,
            return_tensors="pt"
        ).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=self.threshold,
            text_threshold=self.threshold,
            target_sizes=[(h, w)]
        )[0]

        detections = []
        for score, label, box in zip(
            results["scores"].cpu().numpy(),
            results["labels"],
            results["boxes"].cpu().numpy()
        ):
            x1, y1, x2, y2 = box
            detections.append({
                "label": label,
                "score": float(score),
                "box_xyxy": (int(x1), int(y1), int(x2), int(y2)),
                "box_cx_cy_norm": (
                    float((x1 + x2) / 2 / w),
                    float((y1 + y2) / 2 / h)
                )
            })
        return detections


class SAMSegmenter:
    def __init__(self, checkpoint: str, model_type: str = "vit_h", device: str = "cuda"):
        self.device = device if torch.cuda.is_available() else "cpu"
        try:
            from segment_anything import sam_model_registry, SamPredictor
            print(f"[SAM] Loading {model_type} from {checkpoint}...")
            sam = sam_model_registry[model_type](checkpoint=checkpoint).to(self.device)
            self.predictor = SamPredictor(sam)
            self.available = True
            print("[SAM] Ready")
        except Exception as e:
            print(f"[SAM] Not available: {e} — skipping segmentation")
            self.available = False

    def segment_boxes(self, image_bgr: np.ndarray,
                      boxes: List[tuple]) -> List[Optional[np.ndarray]]:
        """Return binary masks for each box_xyxy."""
        if not self.available or not boxes:
            return [None] * len(boxes)

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(image_rgb)
        masks_out = []
        for x1, y1, x2, y2 in boxes:
            box = np.array([x1, y1, x2, y2])
            masks, _, _ = self.predictor.predict(
                box=box,
                multimask_output=False
            )
            masks_out.append(masks[0])
        return masks_out


def draw_detections(image_bgr: np.ndarray, detections: List[Dict]) -> np.ndarray:
    """Draw bounding boxes + labels on image."""
    out = image_bgr.copy()
    for d in detections:
        x1, y1, x2, y2 = d["box_xyxy"]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{d['label']} {d['score']:.2f}"
        cv2.putText(out, label, (x1, max(y1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out
