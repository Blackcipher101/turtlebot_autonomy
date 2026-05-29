"""CLIP embeddings for objects and scene classification."""
import numpy as np
import torch
import cv2
from typing import List, Tuple
from PIL import Image


class CLIPEmbedder:
    def __init__(self, model_name: str = "ViT-B/32", device: str = "cuda"):
        self.device = device if torch.cuda.is_available() else "cpu"
        print(f"[CLIP] Loading {model_name} on {self.device}...")
        try:
            import clip
            self.model, self.preprocess = clip.load(model_name, device=self.device)
            self.backend = "clip"
        except ImportError:
            from transformers import CLIPProcessor, CLIPModel
            self.model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(self.device)
            self.preprocess = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
            self.backend = "transformers"
        self.model.eval()
        print("[CLIP] Ready")

    def embed_texts(self, texts: List[str]) -> np.ndarray:
        """Return (N, D) normalized text embeddings."""
        with torch.no_grad():
            if self.backend == "clip":
                import clip
                tokens = clip.tokenize(texts).to(self.device)
                feats = self.model.encode_text(tokens)
            else:
                inputs = self.preprocess(text=texts, return_tensors="pt",
                                         padding=True).to(self.device)
                feats = self.model.get_text_features(**inputs)
            feats = feats / feats.norm(dim=-1, keepdim=True)
        return feats.cpu().numpy().astype(np.float32)

    def embed_images(self, images_bgr: List[np.ndarray]) -> np.ndarray:
        """Return (N, D) normalized image embeddings from BGR images."""
        pil_images = [Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                      for img in images_bgr]
        with torch.no_grad():
            if self.backend == "clip":
                tensors = torch.stack([self.preprocess(img) for img in pil_images]).to(self.device)
                feats = self.model.encode_image(tensors)
            else:
                inputs = self.preprocess(images=pil_images, return_tensors="pt").to(self.device)
                feats = self.model.get_image_features(**inputs)
            feats = feats / feats.norm(dim=-1, keepdim=True)
        return feats.cpu().numpy().astype(np.float32)

    def classify_scene(self, image_bgr: np.ndarray,
                        scene_labels: List[str]) -> Tuple[str, float]:
        """
        Zero-shot scene classification.
        Returns (best_label, confidence).
        """
        img_emb  = self.embed_images([image_bgr])          # (1, D)
        text_emb = self.embed_texts(scene_labels)           # (N, D)
        sims = (img_emb @ text_emb.T)[0]                    # (N,)
        sims = np.exp(sims) / np.exp(sims).sum()            # softmax
        best_idx = int(np.argmax(sims))
        return scene_labels[best_idx], float(sims[best_idx])

    def embed_label(self, label: str) -> np.ndarray:
        """Single label → (D,) embedding."""
        return self.embed_texts([label])[0]
