from __future__ import annotations

import os
from typing import List

from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

from rag import Chunk

# Load .env from current working dir (or already loaded by main.py)
load_dotenv()


def get_client() -> QdrantClient:
    url = os.getenv("QDRANT_URL")
    key = os.getenv("QDRANT_API_KEY")
    if not url or not key:
        raise RuntimeError("Missing QDRANT_URL or QDRANT_API_KEY in environment/.env")
    return QdrantClient(url=url, api_key=key)


def ensure_collection(collection_name: str, vector_size: int) -> None:
    """
    Create collection if it doesn't exist.
    """
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
    chunks: List[Chunk],
) -> None:
    """
    Store vectors + payload (page, text).
    """
    if len(vectors) != len(chunks):
        raise ValueError("vectors and chunks length mismatch")

    client = get_client()

    points = []
    for idx, (vec, ch) in enumerate(zip(vectors, chunks), start=1):
        payload = {
            "page": ch.meta.get("page"),
            "text": ch.text,
        }
        points.append(models.PointStruct(id=idx, vector=vec, payload=payload))

    # Batch upsert
    client.upsert(collection_name=collection_name, points=points)


def search(
    collection_name: str,
    query_vector: List[float],
    limit: int = 5,
):
    client = get_client()
    return client.search(
        collection_name=collection_name,
        query_vector=query_vector,
        limit=limit,
        with_payload=True,
    )
