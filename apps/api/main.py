from __future__ import annotations

import logging
import os
import re
from pathlib import Path
from typing import List, Tuple

from dotenv import load_dotenv
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from embed import embed_query
from qdrant_store import search as qdrant_search

# ---- Load .env from apps/api/.env no matter where uvicorn is launched ----
load_dotenv(Path(__file__).resolve().with_name(".env"), override=True)

# ---------------- Logging ----------------
logger = logging.getLogger("hackathon_rag_api")
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")

# ---------------- App ----------------
app = FastAPI(title="Hackathon RAG API", version="0.0.3")

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


# ---------------- Config ----------------
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION", "hackathon1_book")


# ---------------- Basic endpoints ----------------
@app.get("/")
def root():
    return {"name": "Master Sahub Hackathon RAG API", "status": "ok"}


@app.get("/health")
def health():
    return {"ok": True, "qdrant_collection": QDRANT_COLLECTION}


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


# ---------------- PDF RAG via Qdrant (vector DB) ----------------

from fastapi import HTTPException

@app.post("/pdf_chat")
def pdf_chat(req: ChatRequest):
    try:
        q = (req.message or "").strip()
        if not q:
            return {"answer": "Empty question.", "sources": []}

        qv = embed_query(q)
        hits = qdrant_search(QDRANT_COLLECTION, qv, limit=5)

        if not hits:
            return {"answer": "No relevant text found in Qdrant.", "sources": []}

        sources = []
        parts = []
        for h in hits:
            payload = h.payload or {}
            page = payload.get("page", "?")
            text = (payload.get("text", "") or "").strip()
            score = float(getattr(h, "score", 0.0))
            sources.append({"page": str(page), "score": score})
            parts.append(f"[p.{page}, score={score:.3f}] {text[:450]}")

        return {"answer": "\n\n---\n\n".join(parts), "sources": sources}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

