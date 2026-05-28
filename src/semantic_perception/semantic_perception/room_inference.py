"""Room-level inference from detected objects via CLIP embedding similarity.

No hardcoded label→room mapping. Room descriptions are embedded at startup,
then compared via cosine similarity to an embedding of the observed objects.
"""

from __future__ import annotations
import logging
from typing import List, Tuple

import numpy as np

from .clip_embedder import CLIPEmbedder

logger = logging.getLogger(__name__)

# Candidate room descriptions — the robot will score any scene against these.
# Add more to extend coverage. Embeddings are computed once at startup.
ROOM_DESCRIPTIONS = [
    "a kitchen with microwave, sink, and refrigerator",
    "a pantry storage room with shelves and boxes",
    "a meeting room or conference room with table and chairs",
    "an office hallway or corridor",
    "an office workspace with desks and monitors",
    "a living room or lounge with sofa",
]

ROOM_LABELS = [
    "kitchen",
    "pantry",
    "meeting_room",
    "hallway",
    "office",
    "lounge",
]


class RoomInferenceEngine:
    def __init__(self, confidence_threshold: float = 0.25):
        self.threshold = confidence_threshold
        self._embedder = CLIPEmbedder()
        self._room_embeddings: np.ndarray | None = None

    def _ensure_embeddings(self):
        if self._room_embeddings is None:
            logger.info("Computing room CLIP embeddings (one-time startup cost)...")
            self._room_embeddings = self._embedder.embed_texts(ROOM_DESCRIPTIONS)
            logger.info("Room embeddings ready.")

    def infer(
        self,
        detected_objects: List[Tuple[str, float]],  # [(label, confidence), ...]
    ) -> Tuple[str, float]:
        """Returns (room_label, confidence) from a list of detected objects."""
        self._ensure_embeddings()

        if not detected_objects:
            return "unknown", 0.0

        # Build a scene description from detected object labels
        object_labels = [label for label, _ in detected_objects if _ > 0.3]
        if not object_labels:
            return "unknown", 0.0

        scene_text = "a room containing " + ", ".join(object_labels[:8])
        scene_emb = self._embedder.embed_texts([scene_text])  # (1, D)

        # Cosine similarity to each room description
        similarities = (scene_emb @ self._room_embeddings.T).squeeze()  # (N,)
        best_idx = int(np.argmax(similarities))
        best_score = float(similarities[best_idx])

        if best_score < self.threshold:
            return "unknown", best_score

        return ROOM_LABELS[best_idx], best_score
