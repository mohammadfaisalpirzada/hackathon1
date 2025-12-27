from __future__ import annotations

from typing import List
from sentence_transformers import SentenceTransformer

# MUST be 768-dim because your Qdrant vectors are dim=768
_model = SentenceTransformer("sentence-transformers/all-mpnet-base-v2")

def embed_texts(texts: List[str]) -> List[List[float]]:
    if not texts:
        return []
    vecs = _model.encode(texts, normalize_embeddings=True, show_progress_bar=False)
    return vecs.tolist()

def embed_query(text: str) -> List[float]:
    return embed_texts([text])[0]
