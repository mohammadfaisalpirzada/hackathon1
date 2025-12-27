from __future__ import annotations

import logging
import os
import re
from pathlib import Path
from typing import List, Tuple

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from rag import Chunk, load_pdf_chunks, search_chunks

# ---------------- Logging ----------------
logger = logging.getLogger("hackathon_rag_api")
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")

# ---------------- App ----------------
app = FastAPI(title="Hackathon RAG API", version="0.0.1")

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

    # quick summary
    if q in {"summarize", "summary"}:
        lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
        return {"answer": "\n".join(lines[:6])}

    # keyword overlap scoring
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


# ---------------- PDF RAG (no LLM yet) ----------------
PDF_NAME = os.getenv("PDF_NAME", "Hackathon I- Physical AI & Humanoid Robotics Textbook.pdf")
PDF_PATH_ENV = os.getenv("PDF_PATH", "").strip()


def _find_pdf_path() -> Path:
    """
    Try common layouts and also current working directory.
    Priority:
      1) PDF_PATH env (exact)
      2) cwd / PDF_NAME
      3) repo guesses based on this file location
    """
    if PDF_PATH_ENV:
        p = Path(PDF_PATH_ENV).expanduser().resolve()
        return p

    # Most common when running uvicorn from repo root
    p0 = (Path.cwd() / PDF_NAME).resolve()
    if p0.exists():
        return p0

    here = Path(__file__).resolve()

    # Common guesses:
    # repo_root/apps/api/main.py  -> parents[2] == repo_root
    candidates = [
        here.parents[2] / PDF_NAME,
        here.parents[3] / PDF_NAME,
        here.parents[1] / PDF_NAME,
        here.parent / PDF_NAME,
    ]
    for c in candidates:
        if c.exists():
            return c.resolve()

    # default expected location
    return candidates[0].resolve()


PDF_PATH: Path = _find_pdf_path()
PDF_CHUNKS: List[Chunk] = []


@app.on_event("startup")
def _startup():
    global PDF_CHUNKS

    logger.info("PDF_NAME = %s", PDF_NAME)
    logger.info("PDF_PATH = %s", str(PDF_PATH))
    logger.info("PDF exists = %s", PDF_PATH.exists())

    if PDF_PATH.exists():
        PDF_CHUNKS = load_pdf_chunks(str(PDF_PATH))
        logger.info("PDF chunks loaded = %d", len(PDF_CHUNKS))
    else:
        PDF_CHUNKS = []
        logger.warning("PDF not found; PDF_CHAT will return 'PDF not loaded'.")


@app.post("/pdf_chat")
def pdf_chat(req: ChatRequest):
    if not PDF_CHUNKS:
        return {
            "answer": (
                "PDF not loaded.\n"
                f"Expected at: {PDF_PATH}\n"
                "Put the PDF in repo root (same level as package.json) and restart API."
            ),
            "sources": [],
        }

    q = (req.message or "").strip()
    if not q:
        return {"answer": "Empty question.", "sources": []}

    # Tiny query boost: only boost if ROS appears as a standalone token (avoid matching 'across', etc.)
    boosted = q
    q_low = q.lower()
    if re.search(r"\bros\s*2\b|\bros2\b|\bros\b", q_low):
        boosted += " ROS2 ROS 2 middleware nodes topics services actions rclcpp rclpy"

    hits = search_chunks(PDF_CHUNKS, boosted, k=5)
    if not hits:
        return {"answer": "No relevant text found in PDF for this question.", "sources": []}

    sources = []
    parts = []
    for h in hits:
        page = h.meta.get("page", "?")
        score = float(h.meta.get("score", 0.0))
        sources.append({"page": page, "score": score})

        snippet = (h.text or "")[:450]
        parts.append(f"[p.{page}, score={score:.3f}] {snippet}")

    return {"answer": "\n\n---\n\n".join(parts), "sources": sources}
