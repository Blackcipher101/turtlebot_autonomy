"""Grounding-DINO wrapper — CPU inference, sparse invocation."""

from __future__ import annotations
import logging
from typing import List, Tuple

import numpy as np
from PIL import Image

logger = logging.getLogger(__name__)

# Prompt covering common indoor objects the robot might see
DEFAULT_PROMPT = (
    "table . chair . microwave . refrigerator . sink . shelf . sofa . "
    "door . cabinet . monitor . keyboard . plant . trash can . whiteboard"
)

DetectionResult = Tuple[str, float, List[float]]  # (label, confidence, [x1,y1,x2,y2])


class GroundingDINOWrapper:
    def __init__(self, confidence_threshold: float = 0.35):
        self.threshold = confidence_threshold
        self._model = None
        self._processor = None
        self._loaded = False

    def _load(self):
        if self._loaded:
            return
        try:
            from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
            import torch
            model_id = "IDEA-Research/grounding-dino-tiny"
            logger.info("Loading Grounding-DINO-tiny (CPU) ...")
            self._processor = AutoProcessor.from_pretrained(model_id)
            self._model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id)
            self._model.eval()
            self._loaded = True
            logger.info("Grounding-DINO loaded.")
        except Exception as e:
            logger.error(f"Failed to load Grounding-DINO: {e}")
            self._loaded = False

    def detect(
        self,
        image: np.ndarray,
        text_prompt: str = DEFAULT_PROMPT,
    ) -> List[DetectionResult]:
        self._load()
        if not self._loaded:
            return []

        try:
            import torch
            pil_img = Image.fromarray(image)
            inputs = self._processor(images=pil_img, text=text_prompt, return_tensors="pt")

            with torch.no_grad():
                outputs = self._model(**inputs)

            results = self._processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                box_threshold=self.threshold,
                text_threshold=self.threshold,
                target_sizes=[pil_img.size[::-1]],
            )[0]

            detections: List[DetectionResult] = []
            for label, score, box in zip(
                results["labels"], results["scores"], results["boxes"]
            ):
                detections.append((
                    str(label),
                    float(score),
                    box.tolist(),  # [x1, y1, x2, y2]
                ))
            return detections

        except Exception as e:
            logger.error(f"Grounding-DINO inference error: {e}")
            return []
