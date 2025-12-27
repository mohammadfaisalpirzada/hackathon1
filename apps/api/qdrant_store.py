from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List

from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

load_dotenv(Path(__file__).resolve().with_name(".env"), override=True)


def get_client() -> QdrantClient:
    url = os.getenv("QDRANT_URL")
    key = os.getenv("QDRANT_API_KEY")
    if not url or not key:
        raise RuntimeError("Missing QDRANT_URL or QDRANT_API_KEY in apps/api/.env")
    return QdrantClient(url=url, api_key=key)


def ensure_collection(collection_name: str, vector_size: int) -> None:
    client = get_client()
    existing = {c.name for c in client.get_collections().collections}
    if collection_name in existing:
        return

    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=vector_size,
            distance=models.Distance.COSINE,
        ),
    )


def upsert_chunks(
    collection_name: str,
    vectors: List[List[float]],
    chunks: List[Any],  # accepts your Chunk dataclass
) -> None:
    if len(vectors) != len(chunks):
        raise ValueError("vectors and chunks length mismatch")

    client = get_client()

    points: List[models.PointStruct] = []
    for idx, (vec, ch) in enumerate(zip(vectors, chunks), start=1):
        payload: Dict[str, Any] = {
            "page": ch.meta.get("page"),
            "text": ch.text,
        }
        points.append(models.PointStruct(id=idx, vector=vec, payload=payload))

    client.upsert(collection_name=collection_name, points=points)


def search(collection_name: str, query_vector: List[float], limit: int = 5):
    client = get_client()
    res = client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=limit,
        with_payload=True,
    )
    return res.points
