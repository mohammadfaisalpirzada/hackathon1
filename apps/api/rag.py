from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

from pypdf import PdfReader
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity


@dataclass
class Chunk:
    text: str
    meta: Dict[str, Any]


def _normalize(s: str) -> str:
    s = (s or "").lower()
    s = re.sub(r"\s+", " ", s).strip()
    return s


def _chunk_text(text: str, chunk_size: int = 900, overlap: int = 150) -> List[str]:
    """
    Simple sliding-window chunker.
    chunk_size/overlap are in characters.
    """
    text = re.sub(r"\s+", " ", text).strip()
    if not text:
        return []
    out: List[str] = []
    i = 0
    step = max(1, chunk_size - overlap)
    while i < len(text):
        out.append(text[i : i + chunk_size])
        i += step
    return out


def load_pdf_chunks(pdf_path: str) -> List[Chunk]:
    reader = PdfReader(pdf_path)
    chunks: List[Chunk] = []

    for page_idx, page in enumerate(reader.pages, start=1):
        raw = (page.extract_text() or "").strip()
        if not raw:
            continue

        for part in _chunk_text(raw):
            chunks.append(Chunk(text=part, meta={"page": page_idx}))

    return chunks


def build_index(chunks: List[Chunk]) -> Tuple[TfidfVectorizer, Any]:
    corpus = [_normalize(c.text) for c in chunks]
    vectorizer = TfidfVectorizer(
        ngram_range=(1, 2),
        min_df=1,
        stop_words="english",
    )
    X = vectorizer.fit_transform(corpus)
    return vectorizer, X


def search_chunks(chunks: List[Chunk], query: str, k: int = 3) -> List[Chunk]:
    if not chunks:
        return []

    q = _normalize(query)
    if not q:
        return []

    vectorizer, X = build_index(chunks)
    qv = vectorizer.transform([q])

    sims = cosine_similarity(qv, X)[0]
    top_idx = sims.argsort()[::-1][:k]

    out: List[Chunk] = []
    for i in top_idx:
        if sims[i] <= 0.0:
            continue
        c = chunks[i]
        c.meta["score"] = float(sims[i])
        out.append(c)

    return out
