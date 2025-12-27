from __future__ import annotations

import os
from pathlib import Path
from typing import List

from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv(Path(__file__).resolve().with_name(".env"), override=True)


def get_client() -> QdrantClient:
    url = os.getenv("QDRANT_URL")
    key = os.getenv("QDRANT_API_KEY")
    if not url or not key:
        raise RuntimeError("Missing QDRANT_URL or QDRANT_API_KEY in apps/api/.env")
    return QdrantClient(url=url, api_key=key)


def search(collection_name: str, query_vector: List[float], limit: int = 5):
    client = get_client()

    # Newer qdrant-client API
    res = client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=limit,
        with_payload=True,
    )
    return res.points
