"""NetworkX-based semantic graph database.

Node types:
  room   — {id, label, pose_x, pose_y, confidence, visit_count, clip_emb}
  object — {id, label, confidence, pose_x, pose_y}

Edge types:
  INSIDE    — object → room
  ADJACENT  — room → room (robot traversed between)
"""

from __future__ import annotations
import json
import logging
import os
from typing import Dict, List, Optional, Tuple

import networkx as nx
import numpy as np

logger = logging.getLogger(__name__)


class SemanticGraphDB:
    def __init__(self, save_path: str = '/ros2_ws/maps/semantic_graph.json'):
        self._g = nx.DiGraph()
        self._save_path = save_path
        self._clip_embedder = None
        self._room_embeddings: Dict[str, np.ndarray] = {}  # node_id → embedding
        self._prev_room_id: Optional[str] = None

        if os.path.exists(save_path):
            self.load(save_path)

    def _get_embedder(self):
        if self._clip_embedder is None:
            try:
                from semantic_perception.clip_embedder import CLIPEmbedder
                self._clip_embedder = CLIPEmbedder()
            except ImportError:
                logger.warning("CLIPEmbedder not available — text queries will use exact matching.")
        return self._clip_embedder

    def add_snapshot(
        self,
        room_label: str,
        room_confidence: float,
        pose_x: float,
        pose_y: float,
        objects: List[Dict],  # [{'label': str, 'confidence': float, 'pose_x': float, 'pose_y': float}]
    ) -> str:
        """Upsert room + objects from a semantic snapshot. Returns room node id."""
        room_id = f"room_{room_label}"

        if self._g.has_node(room_id):
            # Update: average pose, increment visit count, update confidence
            n = self._g.nodes[room_id]
            count = n.get('visit_count', 1)
            n['pose_x'] = (n['pose_x'] * count + pose_x) / (count + 1)
            n['pose_y'] = (n['pose_y'] * count + pose_y) / (count + 1)
            n['visit_count'] = count + 1
            n['confidence'] = max(n['confidence'], room_confidence)
        else:
            self._g.add_node(room_id,
                type='room',
                label=room_label,
                pose_x=pose_x,
                pose_y=pose_y,
                confidence=room_confidence,
                visit_count=1,
            )
            # Compute and store CLIP embedding for this room label
            embedder = self._get_embedder()
            if embedder is not None:
                emb = embedder.embed_texts([room_label])
                self._room_embeddings[room_id] = emb[0]

        # Add ADJACENT edge from previous room
        if self._prev_room_id and self._prev_room_id != room_id:
            if not self._g.has_edge(self._prev_room_id, room_id):
                self._g.add_edge(self._prev_room_id, room_id, type='ADJACENT')
                self._g.add_edge(room_id, self._prev_room_id, type='ADJACENT')
        self._prev_room_id = room_id

        # Add object nodes and INSIDE edges
        for obj in objects:
            obj_id = f"obj_{obj['label']}_{room_label}"
            if not self._g.has_node(obj_id):
                self._g.add_node(obj_id,
                    type='object',
                    label=obj['label'],
                    confidence=obj['confidence'],
                    pose_x=obj.get('pose_x', pose_x),
                    pose_y=obj.get('pose_y', pose_y),
                )
            if not self._g.has_edge(obj_id, room_id):
                self._g.add_edge(obj_id, room_id, type='INSIDE')

        return room_id

    def query_location(self, query_text: str) -> Optional[Tuple[float, float, str, float]]:
        """Returns (x, y, matched_label, confidence) or None.

        Uses CLIP cosine similarity when available, otherwise exact/substring match.
        """
        room_nodes = [
            (nid, data) for nid, data in self._g.nodes(data=True)
            if data.get('type') == 'room'
        ]
        if not room_nodes:
            return None

        embedder = self._get_embedder()
        if embedder is not None and self._room_embeddings:
            query_emb = embedder.embed_texts([query_text])[0]  # (D,)
            best_id, best_score = None, -1.0
            for room_id, _ in room_nodes:
                if room_id not in self._room_embeddings:
                    continue
                score = float(np.dot(query_emb, self._room_embeddings[room_id]))
                if score > best_score:
                    best_score = score
                    best_id = room_id
            if best_id and best_score > 0.2:
                n = self._g.nodes[best_id]
                return n['pose_x'], n['pose_y'], n['label'], best_score
        else:
            # Fallback: substring match on label
            q = query_text.lower()
            for room_id, data in room_nodes:
                if data['label'].lower() in q or q in data['label'].lower():
                    return data['pose_x'], data['pose_y'], data['label'], data['confidence']

        return None

    def known_locations(self) -> List[str]:
        return [
            data['label']
            for _, data in self._g.nodes(data=True)
            if data.get('type') == 'room'
        ]

    def get_room_objects(self, room_label: str) -> List[Dict]:
        room_id = f"room_{room_label}"
        objects = []
        for pred in self._g.predecessors(room_id):
            if self._g.nodes[pred].get('type') == 'object':
                objects.append(dict(self._g.nodes[pred]))
        return objects

    def save(self, path: Optional[str] = None):
        target = path or self._save_path
        os.makedirs(os.path.dirname(target), exist_ok=True)
        data = nx.readwrite.json_graph.node_link_data(self._g)
        with open(target, 'w') as f:
            json.dump(data, f, indent=2)
        logger.info(f"Semantic graph saved to {target}")

    def load(self, path: str):
        try:
            with open(path) as f:
                data = json.load(f)
            self._g = nx.readwrite.json_graph.node_link_graph(data)
            logger.info(f"Semantic graph loaded: {self._g.number_of_nodes()} nodes, "
                        f"{self._g.number_of_edges()} edges")
        except Exception as e:
            logger.error(f"Failed to load semantic graph: {e}")
            self._g = nx.DiGraph()

    @property
    def graph(self) -> nx.DiGraph:
        return self._g
