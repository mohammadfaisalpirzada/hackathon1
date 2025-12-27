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


# ---------------- Text utils ----------------
def _normalize(s: str) -> str:
    s = (s or "").lower()
    s = re.sub(r"\s+", " ", s).strip()
    return s


def _chunk_text(text: str, chunk_size: int = 900, overlap: int = 150) -> List[str]:
    """
    Simple sliding-window chunker (chars-based).
    """
    text = re.sub(r"\s+", " ", (text or "")).strip()
    if not text:
        return []
    out: List[str] = []
    i = 0
    step = max(1, chunk_size - overlap)
    while i < len(text):
        out.append(text[i : i + chunk_size])
        i += step
    return out


def _fix_spaced_letters(text: str) -> str:
    """
    Fix PDFs where words/acronyms come out as 'R O S 2' or 'H u m a n o i d'.

    Heuristic:
      - Join ALL-CAPS sequences length>=2, optionally followed by digits: 'R O S 2' -> 'ROS2'
      - Join any-case sequences length>=3: 'H u m a n o i d' -> 'Humanoid'
    """
    if not text:
        return ""

    # Join ALL-CAPS sequences (and optional trailing digits)
    text = re.sub(
        r"\b(?:[A-Z]\s+){1,}[A-Z](?:\s*\d+)?\b",
        lambda m: m.group(0).replace(" ", ""),
        text,
    )

    # Join any-case sequences (len >= 3 letters)
    text = re.sub(
        r"\b(?:[A-Za-z]\s+){2,}[A-Za-z](?:\s*\d+)?\b",
        lambda m: m.group(0).replace(" ", ""),
        text,
    )

    return text


def preprocess_pdf_text(raw: str, fix_spaced: bool = True) -> str:
    raw = raw or ""
    if fix_spaced:
        raw = _fix_spaced_letters(raw)
    raw = re.sub(r"\s+", " ", raw).strip()
    return raw


# ---------------- PDF loading ----------------
def load_pdf_chunks(pdf_path: str, fix_spaced: bool = True) -> List[Chunk]:
    """
    Extract text from each page, preprocess, chunk, attach meta.page.
    """
    reader = PdfReader(pdf_path)
    chunks: List[Chunk] = []

    for page_idx, page in enumerate(reader.pages, start=1):
        raw = preprocess_pdf_text(page.extract_text() or "", fix_spaced=fix_spaced)
        if not raw:
            continue

        for part in _chunk_text(raw):
            chunks.append(Chunk(text=part, meta={"page": page_idx}))

    return chunks


# ---------------- TF-IDF index ----------------
def build_index(chunks: List[Chunk]) -> Tuple[TfidfVectorizer, Any]:
    """
    Build TF-IDF once and reuse.
    """
    corpus = [_normalize(c.text) for c in chunks]
    vectorizer = TfidfVectorizer(
        ngram_range=(1, 2),
        min_df=1,
        stop_words="english",
    )
    X = vectorizer.fit_transform(corpus)
    return vectorizer, X


def search_chunks_with_index(
    chunks: List[Chunk],
    vectorizer: TfidfVectorizer,
    X: Any,
    query: str,
    k: int = 5,
) -> List[Chunk]:
    """
    Returns top-k chunks with meta['score'] attached.
    Uses a pre-built TF-IDF index (vectorizer + X).
    """
    if not chunks:
        return []

    q = _normalize(query)
    if not q:
        return []

    qv = vectorizer.transform([q])
    sims = cosine_similarity(qv, X)[0]
    top_idx = sims.argsort()[::-1][:k]

    out: List[Chunk] = []
    for i in top_idx:
        if sims[i] <= 0.0:
            continue
        c = chunks[i]
        out.append(
            Chunk(
                text=c.text,
                meta={**c.meta, "score": float(sims[i])},
            )
        )
    return out


# ---------------- Backward-compatible helper ----------------
def search_chunks(chunks: List[Chunk], query: str, k: int = 3) -> List[Chunk]:
    """
    Backward compatible: builds TF-IDF every call (slower).
    Prefer build_index() + search_chunks_with_index().
    """
    if not chunks:
        return []
    vectorizer, X = build_index(chunks)
    return search_chunks_with_index(chunks, vectorizer, X, query, k=k)
