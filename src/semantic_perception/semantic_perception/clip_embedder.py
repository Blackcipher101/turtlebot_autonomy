"""CLIP text/image embedder for room-level semantic inference."""

from __future__ import annotations
import logging
from typing import List

import numpy as np

logger = logging.getLogger(__name__)


class CLIPEmbedder:
    def __init__(self):
        self._model = None
        self._tokenizer = None
        self._loaded = False

    def _load(self):
        if self._loaded:
            return
        try:
            import open_clip
            import torch
            logger.info("Loading CLIP ViT-B/32 (CPU) ...")
            self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                "ViT-B-32", pretrained="openai"
            )
            self._tokenizer = open_clip.get_tokenizer("ViT-B-32")
            self._model.eval()
            self._loaded = True
            logger.info("CLIP loaded.")
        except Exception as e:
            logger.error(f"Failed to load CLIP: {e}")

    def embed_texts(self, texts: List[str]) -> np.ndarray:
        """Returns (N, D) normalized text embeddings."""
        self._load()
        if not self._loaded:
            return np.zeros((len(texts), 512))
        import torch
        tokens = self._tokenizer(texts)
        with torch.no_grad():
            feats = self._model.encode_text(tokens)
            feats = feats / feats.norm(dim=-1, keepdim=True)
        return feats.numpy()

    def embed_image(self, image_rgb: np.ndarray) -> np.ndarray:
        """Returns (1, D) normalized image embedding."""
        self._load()
        if not self._loaded:
            return np.zeros((1, 512))
        import torch
        from PIL import Image
        pil = Image.fromarray(image_rgb)
        tensor = self._preprocess(pil).unsqueeze(0)
        with torch.no_grad():
            feat = self._model.encode_image(tensor)
            feat = feat / feat.norm(dim=-1, keepdim=True)
        return feat.numpy()
