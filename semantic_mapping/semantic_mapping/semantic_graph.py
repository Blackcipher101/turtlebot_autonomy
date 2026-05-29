"""
Semantic graph.
Nodes  = segmented objects with image CLIP embedding + 3D position + scene type.
Edges  = spatial proximity OR semantic similarity.
Query  = CLIP text embedding → cosine search → nearest node → 3D position.
"""
import numpy as np
import networkx as nx
import pickle
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field


@dataclass
class SemanticNode:
    id: int
    position: np.ndarray          # world [x, y, z]
    clip_image_emb: np.ndarray    # (D,) CLIP image embedding of the crop
    scene_type: str               # "garden", "corridor", etc.
    blip_caption: str             # "a red double-decker bus"  (from BLIP-2, optional)
    score: float                  # SAM stability/iou score
    frame_id: int
    timestamp: float
    bbox: tuple                   # pixel bbox in original image (x,y,w,h)
    area: int                     # pixel area — useful for filtering
    observations: int = 1


class SemanticGraph:
    def __init__(self, spatial_edge_dist: float = 5.0,
                 sem_edge_threshold: float = 0.75):
        self.G = nx.Graph()
        self.nodes: List[SemanticNode] = []
        self.spatial_edge_dist  = spatial_edge_dist
        self.sem_edge_threshold = sem_edge_threshold
        self._counter = 0

    # ── Adding nodes ──────────────────────────────────────────────────────────

    def add_node(self, node: SemanticNode) -> int:
        node.id = self._counter
        self._counter += 1
        self.nodes.append(node)
        self.G.add_node(node.id,
                        position=node.position,
                        scene_type=node.scene_type,
                        caption=node.blip_caption,
                        score=node.score)
        self._connect_edges(node)
        return node.id

    def _connect_edges(self, new: SemanticNode):
        for old in self.nodes[:-1]:
            dist = float(np.linalg.norm(new.position - old.position))
            if dist < self.spatial_edge_dist:
                self.G.add_edge(new.id, old.id, type="spatial", weight=1.0 / (dist + 1e-3))

            sim = float(new.clip_image_emb @ old.clip_image_emb)
            if sim > self.sem_edge_threshold:
                self.G.add_edge(new.id, old.id, type="semantic", weight=sim)

    # ── Merging ───────────────────────────────────────────────────────────────

    def merge_duplicates(self, dist_m: float = 1.5, sim: float = 0.88):
        """Merge nodes that are both spatially close AND visually similar."""
        merged = set()
        for i, a in enumerate(self.nodes):
            if a.id in merged:
                continue
            for b in self.nodes[i + 1:]:
                if b.id in merged:
                    continue
                if np.linalg.norm(a.position - b.position) < dist_m and \
                   float(a.clip_image_emb @ b.clip_image_emb) > sim:
                    # Average position, keep better caption
                    w = a.observations
                    a.position = (a.position * w + b.position) / (w + 1)
                    a.observations += b.observations
                    if not a.blip_caption and b.blip_caption:
                        a.blip_caption = b.blip_caption
                    merged.add(b.id)

        self.nodes = [n for n in self.nodes if n.id not in merged]
        for nid in merged:
            if self.G.has_node(nid):
                self.G.remove_node(nid)
        print(f"[Graph] After merge: {len(self.nodes)} nodes ({len(merged)} removed)")

    # ── Querying ──────────────────────────────────────────────────────────────

    def query(self, text: str, clip_embedder,
              scene_filter: Optional[str] = None,
              top_k: int = 5) -> List[Tuple[float, "SemanticNode"]]:
        """
        Natural language query → ranked (score, node) list.

        Scoring:
          - CLIP text↔image cosine similarity  (primary)
          - BLIP caption text match bonus       (secondary)
          - scene type match bonus              (secondary)
        """
        if not self.nodes:
            return []

        text_emb = clip_embedder.embed_texts([text])[0]   # (D,)

        results = []
        for node in self.nodes:
            # Skip wrong scene if filter given
            if scene_filter and scene_filter.lower() not in node.scene_type.lower():
                continue

            clip_sim = float(text_emb @ node.clip_image_emb)

            # BLIP caption bonus — simple word overlap
            caption_bonus = 0.0
            if node.blip_caption:
                query_words  = set(text.lower().split())
                caption_words = set(node.blip_caption.lower().split())
                overlap = len(query_words & caption_words)
                caption_bonus = 0.05 * min(overlap, 3)

            # Scene bonus — check each word of query against scene_type
            scene_bonus = 0.0
            for word in text.lower().split():
                if word in node.scene_type.lower():
                    scene_bonus = 0.08
                    break

            score = clip_sim + caption_bonus + scene_bonus
            results.append((score, node))

        results.sort(key=lambda x: -x[0])
        return results[:top_k]

    def get_navigation_target(self, text: str, clip_embedder,
                               min_sim: float = 0.20) -> Optional[np.ndarray]:
        """
        Main entry point for robot: text → 3D world position.
        Returns None if nothing found above min_sim threshold.
        """
        results = self.query(text, clip_embedder, top_k=1)
        if not results:
            print(f"[Graph] No results for '{text}'")
            return None
        score, node = results[0]
        if score < min_sim:
            print(f"[Graph] Best match too weak ({score:.3f}) for '{text}'")
            return None
        caption = f" ('{node.blip_caption}')" if node.blip_caption else ""
        print(f"[Graph] '{text}' → node {node.id}{caption} "
              f"@ {node.position} scene={node.scene_type} score={score:.3f}")
        return node.position

    # ── Stats / IO ────────────────────────────────────────────────────────────

    def stats(self) -> Dict:
        scenes = [n.scene_type for n in self.nodes]
        return {
            "nodes":   len(self.nodes),
            "edges":   self.G.number_of_edges(),
            "scenes":  {s: scenes.count(s) for s in set(scenes)},
            "captioned": sum(1 for n in self.nodes if n.blip_caption),
        }

    def save(self, path: str):
        with open(path, "wb") as f:
            pickle.dump(self, f)
        print(f"[Graph] Saved → {path}")

    @staticmethod
    def load(path: str) -> "SemanticGraph":
        with open(path, "rb") as f:
            return pickle.load(f)

    # ── Visualization ─────────────────────────────────────────────────────────

    def visualize_2d(self, save_path: Optional[str] = None,
                     color_by: str = "scene"):
        """
        Top-down 2D map.
        color_by: "scene" (color nodes by scene type) or "cluster" (by community).
        """
        import matplotlib.pyplot as plt
        import matplotlib.cm as cm

        if not self.nodes:
            print("[Graph] Nothing to visualize")
            return

        fig, ax = plt.subplots(figsize=(14, 12))

        if color_by == "scene":
            labels = sorted(set(n.scene_type for n in self.nodes))
            cmap   = cm.tab20(np.linspace(0, 1, max(len(labels), 1)))
            cdict  = {l: cmap[i] for i, l in enumerate(labels)}
        else:
            # Community detection coloring
            communities = list(nx.community.greedy_modularity_communities(self.G))
            node_comm   = {}
            for i, comm in enumerate(communities):
                for nid in comm:
                    node_comm[nid] = i
            cmap  = cm.tab20(np.linspace(0, 1, max(len(communities), 1)))

        # Draw spatial edges
        for u, v, d in self.G.edges(data=True):
            if d.get("type") == "spatial":
                nu = next((n for n in self.nodes if n.id == u), None)
                nv = next((n for n in self.nodes if n.id == v), None)
                if nu and nv:
                    ax.plot([nu.position[0], nv.position[0]],
                            [nu.position[1], nv.position[1]],
                            color="lightblue", alpha=0.3, linewidth=0.6, zorder=1)

        # Draw semantic edges
        for u, v, d in self.G.edges(data=True):
            if d.get("type") == "semantic":
                nu = next((n for n in self.nodes if n.id == u), None)
                nv = next((n for n in self.nodes if n.id == v), None)
                if nu and nv:
                    ax.plot([nu.position[0], nv.position[0]],
                            [nu.position[1], nv.position[1]],
                            color="orange", alpha=0.4, linewidth=0.8, zorder=1)

        # Draw nodes
        for node in self.nodes:
            if color_by == "scene":
                c = cdict.get(node.scene_type, "gray")
            else:
                c = cmap[node_comm.get(node.id, 0)]

            size = max(20, min(200, node.area / 500))
            ax.scatter(node.position[0], node.position[1],
                       c=[c], s=size, zorder=3, alpha=0.85)

            caption = node.blip_caption[:20] if node.blip_caption else ""
            if caption:
                ax.annotate(caption, (node.position[0], node.position[1]),
                            fontsize=5, alpha=0.7, ha="center",
                            xytext=(0, 5), textcoords="offset points")

        # Legend
        if color_by == "scene":
            for label, color in cdict.items():
                count = sum(1 for n in self.nodes if n.scene_type == label)
                ax.scatter([], [], c=[color], s=50, label=f"{label} ({count})")
            ax.legend(loc="upper right", fontsize=7, ncol=2)

        # Edge legend
        from matplotlib.lines import Line2D
        ax.legend(handles=[
            Line2D([0], [0], color="lightblue", label="Spatial edge"),
            Line2D([0], [0], color="orange",    label="Semantic edge"),
        ] + ([ax.scatter([], [], c=[cdict[l]], s=40, label=f"{l}")
              for l in cdict] if color_by == "scene" else []),
            loc="upper right", fontsize=7, ncol=2)

        ax.set_title(f"Semantic Map  |  {len(self.nodes)} objects  |  "
                     f"{self.G.number_of_edges()} edges", fontsize=12)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"[Graph] Map saved → {save_path}")
        else:
            plt.show()
        plt.close()
