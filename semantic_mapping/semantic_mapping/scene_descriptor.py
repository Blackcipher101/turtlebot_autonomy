"""
Scene-level understanding using BLIP-2 (caption) + CLIP (classification).
Given a full image → natural language description + scene type label.
"""
import numpy as np
import torch
import cv2
from PIL import Image
from typing import Tuple, List


class SceneDescriptor:
    def __init__(self, device: str = "cuda"):
        self.device = device if torch.cuda.is_available() else "cpu"
        print(f"[Scene] Loading BLIP-2 on {self.device}...")
        from transformers import Blip2Processor, Blip2ForConditionalGeneration
        self.processor = Blip2Processor.from_pretrained("Salesforce/blip2-opt-2.7b")
        self.model = Blip2ForConditionalGeneration.from_pretrained(
            "Salesforce/blip2-opt-2.7b",
            torch_dtype=torch.float16 if self.device == "cuda" else torch.float32
        ).to(self.device)
        self.model.eval()
        print("[Scene] BLIP-2 ready")

    def describe(self, image_bgr: np.ndarray) -> str:
        """Return a natural language caption of the scene."""
        pil = Image.fromarray(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB))
        inputs = self.processor(images=pil, return_tensors="pt").to(
            self.device, torch.float16 if self.device == "cuda" else torch.float32
        )
        with torch.no_grad():
            ids = self.model.generate(**inputs, max_new_tokens=50)
        return self.processor.batch_decode(ids, skip_special_tokens=True)[0].strip()

    def describe_crop(self, crop_bgr: np.ndarray) -> str:
        """Describe a single segmented object crop."""
        if crop_bgr.shape[0] < 8 or crop_bgr.shape[1] < 8:
            return ""
        # Pad small crops so BLIP doesn't struggle
        h, w = crop_bgr.shape[:2]
        if h < 64 or w < 64:
            scale = max(64 / h, 64 / w)
            crop_bgr = cv2.resize(crop_bgr, (int(w * scale), int(h * scale)))
        return self.describe(crop_bgr)


class CLIPSceneClassifier:
    """Fast CLIP-based scene type classification (no heavy captioning model)."""
    SCENE_LABELS = [
        "garden", "indoor room", "office", "kitchen", "corridor",
        "urban street", "parking lot", "park", "construction site",
        "living room", "warehouse", "campus outdoor", "forest"
    ]

    def __init__(self, clip_embedder):
        self.clip = clip_embedder
        # Pre-embed all scene labels once
        self._scene_embs = self.clip.embed_texts(self.SCENE_LABELS)  # (N, D)

    def classify(self, image_bgr: np.ndarray) -> Tuple[str, float]:
        """Returns (scene_label, confidence)."""
        img_emb = self.clip.embed_images([image_bgr])   # (1, D)
        sims = (img_emb @ self._scene_embs.T)[0]        # (N,)
        probs = np.exp(sims) / np.exp(sims).sum()
        best = int(np.argmax(probs))
        return self.SCENE_LABELS[best], float(probs[best])
