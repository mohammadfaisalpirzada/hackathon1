from __future__ import annotations

import logging
import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from dotenv import load_dotenv
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from embed import embed_query
from qdrant_store import search as qdrant_search

# ✅ New Gemini SDK
from google import genai
from google.genai import types
from google.genai.errors import ClientError


# ---- Load .env from apps/api/.env no matter where uvicorn is launched ----
load_dotenv(Path(__file__).resolve().with_name(".env"), override=True)

# ---------------- Config ----------------
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION", "hackathon1_book_v2").strip()
TOP_K = int(os.getenv("TOP_K", "5"))
SCORE_THRESHOLD = float(os.getenv("SCORE_THRESHOLD", "0.25"))

MAX_CONTEXT_CHARS = int(os.getenv("MAX_CONTEXT_CHARS", "8000"))
MAX_CHUNK_CHARS = int(os.getenv("MAX_CHUNK_CHARS", "1200"))

GEMINI_KEY = (os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY") or "").strip()
# IMPORTANT: you fixed quota by using 2.5-flash. Keep that as default.
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-flash").strip()

# Optional fallback model if primary fails (e.g., quota issues on one model)
GEMINI_FALLBACK_MODEL = os.getenv("GEMINI_FALLBACK_MODEL", "").strip()

# Generation controls
GEMINI_TEMPERATURE = float(os.getenv("GEMINI_TEMPERATURE", "0.2"))
GEMINI_MAX_OUTPUT_TOKENS = int(os.getenv("GEMINI_MAX_OUTPUT_TOKENS", "600"))

gemini_client: Optional[genai.Client] = None
if GEMINI_KEY:
    gemini_client = genai.Client(api_key=GEMINI_KEY)

# ---------------- Logging ----------------
logger = logging.getLogger("hackathon_rag_api")
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")

# ---------------- App ----------------
app = FastAPI(title="Hackathon RAG API", version="0.0.8")

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
    message: str = Field(..., min_length=1)


class AskOnTextRequest(BaseModel):
    selected_text: str
    question: str


class HitOut(BaseModel):
    page: str = "?"
    score: float = 0.0
    text: str = ""


class RagResponse(BaseModel):
    ok: bool = True
    collection: str = QDRANT_COLLECTION
    threshold: float = SCORE_THRESHOLD
    hits: List[HitOut] = []
    answer: str = ""
    error: str = ""
    llm_error: str = ""


# ---------------- Basic endpoints ----------------
@app.get("/")
def root():
    return {"name": "Hackathon RAG API", "status": "ok"}


@app.get("/health")
def health():
    return {
        "ok": True,
        "qdrant_collection": QDRANT_COLLECTION,
        "score_threshold": SCORE_THRESHOLD,
        "top_k": TOP_K,
        "gemini_key_present": bool(GEMINI_KEY),
        "gemini_client_created": bool(gemini_client),
        "gemini_model": GEMINI_MODEL if gemini_client else None,
        "gemini_fallback_model": GEMINI_FALLBACK_MODEL or None,
        "max_context_chars": MAX_CONTEXT_CHARS,
        "max_chunk_chars": MAX_CHUNK_CHARS,
        "gemini_max_output_tokens": GEMINI_MAX_OUTPUT_TOKENS,
    }


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


# ---------------- Helpers ----------------
def _maybe_boost_query(q: str) -> str:
    boosted = q
    if re.search(r"\bros\s*2\b|\bros2\b|\bros\b", q.lower()):
        boosted += " ROS2 ROS 2 architecture middleware nodes topics services actions rclcpp rclpy"
    return boosted


def _safe_embed(q: str) -> Tuple[Optional[List[float]], str]:
    """
    Never silently returns junk.
    """
    try:
        v = embed_query(q)
        if not isinstance(v, list) or not v:
            return None, "Embedding returned empty vector."
        if all(abs(x) < 1e-12 for x in v):
            return None, "Embedding vector is all zeros (embedder likely failing)."
        return v, ""
    except Exception as e:
        logger.exception("Embedding failed: %s", e)
        return None, f"Embedding failed: {e}"


def _raw_scores(hits: List[Any]) -> List[Tuple[float, Any]]:
    return [
        (float(getattr(h, "score", 0.0)), (getattr(h, "payload", {}) or {}).get("page"))
        for h in hits
    ]


def _filter_hits(hits: List[Any], min_score: float) -> List[Any]:
    return [h for h in hits if float(getattr(h, "score", 0.0)) >= min_score]


def _to_hitout(h: Any) -> HitOut:
    payload: Dict[str, Any] = getattr(h, "payload", {}) or {}
    page = str(payload.get("page", "?"))
    text = (payload.get("text", "") or "").strip()
    score = float(getattr(h, "score", 0.0))
    if len(text) > MAX_CHUNK_CHARS:
        text = text[:MAX_CHUNK_CHARS]
    return HitOut(page=page, score=score, text=text)


def _build_context(hits_out: List[HitOut]) -> str:
    parts: List[str] = []
    total = 0
    for h in hits_out:
        block = f"[p.{h.page}] {h.text}".strip()
        if not block:
            continue
        if total + len(block) > MAX_CONTEXT_CHARS:
            break
        parts.append(block)
        total += len(block)
    return "\n\n".join(parts)


def _prompt_for_answer(question: str, context: str) -> str:
    """
    Strict format so the model doesn't cut off mid-list.
    """
    return (
        "Answer using ONLY the context below.\n"
        "If something is not in the context, say: Not found in the provided pages.\n\n"
        "Return format (follow exactly):\n"
        "1) Where ROS 2 is mentioned (pages)\n"
        "2) Topics included (bullet list)\n\n"
        f"Question: {question}\n\n"
        f"Context:\n{context}\n"
    )


def _call_gemini(model_name: str, prompt: str) -> str:
    """
    Call Gemini once. Raises on error.
    """
    if gemini_client is None:
        raise RuntimeError("Gemini not configured.")

    resp = gemini_client.models.generate_content(
        model=model_name,
        contents=prompt,
        config=types.GenerateContentConfig(
            temperature=GEMINI_TEMPERATURE,
            max_output_tokens=GEMINI_MAX_OUTPUT_TOKENS,
        ),
    )
    return (resp.text or "").strip()


def _gemini_answer(question: str, context: str) -> Tuple[str, str]:
    """
    Returns (answer, llm_error). Never raises.
    Tries GEMINI_MODEL first, then GEMINI_FALLBACK_MODEL if set.
    """
    if gemini_client is None:
        return "", "Gemini not configured."

    prompt = _prompt_for_answer(question, context)

    try:
        ans = _call_gemini(GEMINI_MODEL, prompt)
        if not ans:
            return "", "Gemini returned empty text."
        return ans, ""
    except ClientError as e:
        # If primary fails, optionally retry fallback
        logger.exception("Gemini ClientError (model=%s): %s", GEMINI_MODEL, e)
        if GEMINI_FALLBACK_MODEL:
            try:
                ans = _call_gemini(GEMINI_FALLBACK_MODEL, prompt)
                if not ans:
                    return "", f"Primary failed; fallback returned empty text. Primary error: {e}"
                return ans, ""
            except Exception as e2:
                logger.exception("Gemini fallback failed (model=%s): %s", GEMINI_FALLBACK_MODEL, e2)
                return "", f"Primary error: {e} | Fallback error: {e2}"
        return "", str(e)
    except Exception as e:
        logger.exception("Gemini failed (model=%s): %s", GEMINI_MODEL, e)
        return "", str(e)


# ---------------- PDF RAG via Qdrant: snippets ----------------
@app.post("/pdf_chat", response_model=RagResponse)
def pdf_chat(req: ChatRequest) -> RagResponse:
    q = (req.message or "").strip()
    if not q:
        return RagResponse(ok=False, answer="Empty question.", error="Empty message.")

    boosted = _maybe_boost_query(q)

    qv, emb_err = _safe_embed(boosted)
    if not qv:
        return RagResponse(ok=False, answer="Embedding failed.", error=emb_err)

    hits = qdrant_search(QDRANT_COLLECTION, qv, limit=TOP_K) or []
    logger.info("RAW hits (score,page): %s", _raw_scores(hits))

    filtered = _filter_hits(hits, min_score=SCORE_THRESHOLD)
    logger.info("FILTERED hits count=%d threshold=%.3f", len(filtered), SCORE_THRESHOLD)

    if not filtered:
        return RagResponse(ok=True, hits=[], answer="No relevant text found.")

    hits_out = [_to_hitout(h) for h in filtered]
    parts = [f"[p.{h.page}, score={h.score:.3f}] {h.text}" for h in hits_out]
    answer = "\n\n---\n\n".join(parts)

    return RagResponse(ok=True, hits=hits_out, answer=answer)


# ---------------- PDF RAG via Qdrant: clean final answer (Gemini) ----------------
@app.post("/pdf_answer", response_model=RagResponse)
def pdf_answer(req: ChatRequest) -> RagResponse:
    q = (req.message or "").strip()
    if not q:
        return RagResponse(ok=False, answer="Empty question.", error="Empty message.")

    boosted = _maybe_boost_query(q)

    qv, emb_err = _safe_embed(boosted)
    if not qv:
        return RagResponse(ok=False, answer="Embedding failed.", error=emb_err)

    hits = qdrant_search(QDRANT_COLLECTION, qv, limit=TOP_K) or []
    logger.info("RAW hits (score,page): %s", _raw_scores(hits))

    filtered = _filter_hits(hits, min_score=SCORE_THRESHOLD)
    logger.info("FILTERED hits count=%d threshold=%.3f", len(filtered), SCORE_THRESHOLD)

    if not filtered:
        return RagResponse(ok=True, hits=[], answer="No relevant text found.")

    hits_out = [_to_hitout(h) for h in filtered]
    context = _build_context(hits_out)

    # Always available fallback (useful for debugging + UI sources)
    fallback = "\n\n---\n\n".join([f"[p.{h.page}] {h.text}" for h in hits_out])[:2500]

    ans, llm_err = _gemini_answer(q, context)
    if ans:
        return RagResponse(ok=True, hits=hits_out, answer=ans)
    return RagResponse(ok=True, hits=hits_out, answer=fallback, llm_error=llm_err)
