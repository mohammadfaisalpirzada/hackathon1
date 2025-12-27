from __future__ import annotations

import os
from pathlib import Path
from typing import List

from dotenv import load_dotenv
from google import genai

load_dotenv(Path(__file__).resolve().with_name(".env"), override=True)

api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise RuntimeError("GEMINI_API_KEY missing in apps/api/.env")

client = genai.Client(api_key=api_key)

# Gemini embedding model (common choice)
EMBED_MODEL = os.getenv("GEMINI_EMBED_MODEL", "text-embedding-004")


def embed_texts(texts: List[str]) -> List[List[float]]:
    out: List[List[float]] = []
    for t in texts:
        r = client.models.embed_content(
            model=EMBED_MODEL,
            contents=t,
        )
        out.append(r.embeddings[0].values)
    return out


def embed_query(text: str) -> List[float]:
    return embed_texts([text])[0]
