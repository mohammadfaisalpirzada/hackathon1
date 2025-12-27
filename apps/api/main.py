from pathlib import Path
from dotenv import load_dotenv
load_dotenv(Path(__file__).resolve().with_name(".env"))



from __future__ import annotations

import logging
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple, Optional

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from pypdf import PdfReader
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity

# Import Chunk from rag.py (keep your existing dataclass there)
from rag import Chunk

# ---------------- Logging ----------------
logger = logging.getLogger("hackathon_rag_api")
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")

# ---------------- App ----------------
app = FastAPI(title="Hackathon RAG API", version="0.0.2")

# ---------------- CORS ----------------
ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
    "http://localhost:8000",
    "http://127.0.0.1:8000",
    "https://mohammadfaisalpirzada.github.io",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------- Models ----------------
class ChatRequest(BaseModel):
    message: str


class AskOnTextRequest(BaseModel):
    selected_text: str
    question: str


# ---------------- Helpers (text + chunking) ----------------
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
      - Join ALL-CAPS sequences (len>=2), optionally followed by digits: "R O S 2" -> "ROS2"
      - Join any-case sequences (len>=3): "H u m a n o i d" -> "Humanoid"
    """
    if not text:
        return ""

    # Join ALL-CAPS sequences (with optional digits)
    text = re.sub(
        r"\b(?:[A-Z]\s+){1,}[A-Z](?:\s*\d+)?\b",
        lambda m: m.group(0).replace(" ", ""),
        text,
    )

    # Join any-case sequences (len>=3 letters)
    text = re.sub(
        r"\b(?:[A-Za-z]\s+){2,}[A-Za-z](?:\s*\d+)?\b",
        lambda m: m.group(0).replace(" ", ""),
        text,
    )

    return text


def _preprocess_pdf_text(raw: str, fix_spaced: bool = True) -> str:
    raw = raw or ""
    if fix_spaced:
        raw = _fix_spaced_letters(raw)
    raw = re.sub(r"\s+", " ", raw).strip()
    return raw


def load_pdf_chunks_local(pdf_path: str, fix_spaced: bool = True) -> List[Chunk]:
    """
    Local PDF loader (so main.py works even if rag.py still has the older loader).
    """
    reader = PdfReader(pdf_path)
    chunks: List[Chunk] = []

    for page_idx, page in enumerate(reader.pages, start=1):
        raw = _preprocess_pdf_text(page.extract_text() or "", fix_spaced=fix_spaced)
        if not raw:
            continue

        for part in _chunk_text(raw):
            chunks.append(Chunk(text=part, meta={"page": page_idx}))

    return chunks


def build_index_once(chunks: List[Chunk]) -> Tuple[TfidfVectorizer, Any]:
    corpus = [_normalize(c.text) for c in chunks]
    vectorizer = TfidfVectorizer(
        ngram_range=(1, 2),
        min_df=1,
        stop_words="english",
    )
    X = vectorizer.fit_transform(corpus)
    return vectorizer, X


def search_chunks_with_index(
    chunks: List[Chunk], vectorizer: TfidfVectorizer, X: Any, query: str, k: int = 5
) -> List[Chunk]:
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
        c2 = Chunk(text=c.text, meta=dict(c.meta))
        c2.meta["score"] = float(sims[i])
        out.append(c2)
    return out


# ---------------- Basic endpoints ----------------
@app.get("/")
def root():
    return {"name": "Master Sahub Hackathon RAG API", "status": "ok"}


@app.get("/health")
def health():
    return {"ok": True}


@app.post("/chat")
def chat(req: ChatRequest):
    return {"answer": f"Echo: {req.message}"}


# ---------------- Selected-text answering (no LLM) ----------------
@app.post("/ask_on_text")
def ask_on_text(req: AskOnTextRequest):
    text = (req.selected_text or "").strip()
    q = (req.question or "").strip().lower()

    if not text:
        return {"answer": "No selected text provided."}
    if not q:
        return {"answer": "No question provided."}

    if q in {"summarize", "summary"}:
        lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
        return {"answer": "\n".join(lines[:6])}

    words = [w for w in re.findall(r"[a-zA-Z0-9\-]+", q) if len(w) > 2]
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if not lines:
        return {"answer": text[:800]}

    scored: List[Tuple[int, str]] = []
    for ln in lines:
        ln_low = ln.lower()
        score = sum(1 for w in words if w in ln_low)
        scored.append((score, ln))

    scored.sort(key=lambda x: x[0], reverse=True)
    best_score, _ = scored[0]

    if best_score == 0:
        return {"answer": "\n".join(lines[:2])}

    top_lines = [ln for s, ln in scored[:2] if s > 0]
    return {"answer": "\n".join(top_lines)}


# ---------------- PDF RAG (TF-IDF, cached) ----------------
PDF_NAME = os.getenv("PDF_NAME", "Hackathon I- Physical AI & Humanoid Robotics Textbook.pdf")
PDF_PATH_ENV = os.getenv("PDF_PATH", "").strip()

PDF_PATH: Path
PDF_CHUNKS: List[Chunk] = []
VEC: Optional[TfidfVectorizer] = None
X = None


def _find_pdf_path() -> Path:
    """
    Priority:
      1) PDF_PATH env (exact)
      2) cwd / PDF_NAME
      3) repo guesses based on this file location
    """
    if PDF_PATH_ENV:
        return Path(PDF_PATH_ENV).expanduser().resolve()

    p0 = (Path.cwd() / PDF_NAME).resolve()
    if p0.exists():
        return p0

    here = Path(__file__).resolve()
    candidates = [
        here.parents[2] / PDF_NAME,  # repo_root
        here.parents[3] / PDF_NAME,
        here.parents[1] / PDF_NAME,
        here.parent / PDF_NAME,
    ]
    for c in candidates:
        if c.exists():
            return c.resolve()

    return candidates[0].resolve()


PDF_PATH = _find_pdf_path()


@app.on_event("startup")
def _startup():
    global PDF_CHUNKS, VEC, X

    logger.info("PDF_NAME = %s", PDF_NAME)
    logger.info("PDF_PATH = %s", str(PDF_PATH))
    logger.info("PDF exists = %s", PDF_PATH.exists())

    if not PDF_PATH.exists():
        PDF_CHUNKS = []
        VEC = None
        X = None
        logger.warning("PDF not found; /pdf_chat will return 'PDF not loaded'.")
        return

    # Load chunks with spaced-letter fix ON (key improvement)
    PDF_CHUNKS = load_pdf_chunks_local(str(PDF_PATH), fix_spaced=True)
    logger.info("PDF chunks loaded = %d", len(PDF_CHUNKS))

    if not PDF_CHUNKS:
        VEC = None
        X = None
        logger.warning("No chunks extracted. PDF may be scanned/image-only.")
        return

    # Build index once
    VEC, X = build_index_once(PDF_CHUNKS)
    logger.info("TF-IDF index built.")

    # Quick sanity checks
    sample_text = " ".join(c.text for c in PDF_CHUNKS[:25]).lower()
    logger.info("Sanity: contains 'ros' = %s", ("ros" in sample_text))
    logger.info("Sanity: contains 'gazebo' = %s", ("gazebo" in sample_text))


@app.post("/pdf_chat")
def pdf_chat(req: ChatRequest):
    if not PDF_CHUNKS or VEC is None or X is None:
        return {
            "answer": (
                "PDF not loaded or no text extracted.\n"
                f"Expected at: {PDF_PATH}\n"
                "If the PDF is scanned (image-only), you must OCR it or use a different extractor."
            ),
            "sources": [],
        }

    q = (req.message or "").strip()
    if not q:
        return {"answer": "Empty question.", "sources": []}

    # Tiny query boost
    boosted = q
    q_low = q.lower()
    if re.search(r"\bros\s*2\b|\bros2\b|\bros\b", q_low):
        boosted += " ROS2 ROS 2 middleware nodes topics services actions rclcpp rclpy gazebo isaac"

    hits = search_chunks_with_index(PDF_CHUNKS, VEC, X, boosted, k=5)
    if not hits:
        return {"answer": "No relevant text found in PDF for this question.", "sources": []}

    sources = []
    parts = []
    for h in hits:
        page = h.meta.get("page", "?")
        score = float(h.meta.get("score", 0.0))
        sources.append({"page": str(page), "score": score})

        snippet = (h.text or "")[:450]
        parts.append(f"[p.{page}, score={score:.3f}] {snippet}")

    return {"answer": "\n\n---\n\n".join(parts), "sources": sources}
