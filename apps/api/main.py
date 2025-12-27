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
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-2.0-flash").strip()

gemini_client: Optional[genai.Client] = None
if GEMINI_KEY:
    gemini_client = genai.Client(api_key=GEMINI_KEY)

# ---------------- Logging ----------------
logger = logging.getLogger("hackathon_rag_api")
logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")

# ---------------- App ----------------
app = FastAPI(title="Hackathon RAG API", version="0.0.7")

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
        # Log minimal debug to confirm it's sane
        logger.info("embed len=%d first5=%s", len(v), v[:5])
        return v, ""
    except Exception as e:
        logger.exception("Embedding failed: %s", e)
        return None, f"Embedding failed: {e}"


def _raw_scores(hits: List[Any]) -> List[Tuple[float, Any]]:
    out: List[Tuple[float, Any]] = []
    for h in hits:
        out.append((float(getattr(h, "score", 0.0)), (getattr(h, "payload", {}) or {}).get("page")))
    return out


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


def _gemini_summarize(question: str, context: str) -> Tuple[str, str]:
    """
    Returns (answer, llm_error). Never raises.
    """
    if gemini_client is None:
        return "", "Gemini not configured."

    prompt = (
        "You are answering using ONLY the provided textbook context.\n"
        "Rules:\n"
        "- Use ONLY the context.\n"
        "- If the answer is not in the context, say: Not found in the provided pages.\n"
        "- Keep it short and clear (5-8 lines).\n\n"
        f"Question: {question}\n\n"
        "Context:\n"
        f"{context}\n"
    )

    try:
        resp = gemini_client.models.generate_content(
            model=GEMINI_MODEL,
            contents=prompt,
            config=types.GenerateContentConfig(
                temperature=0.2,
                max_output_tokens=350,
            ),
        )
        ans = (resp.text or "").strip()
        if not ans:
            return "", "Gemini returned empty text."
        return ans, ""
    except ClientError as e:
        logger.exception("Gemini ClientError: %s", e)
        return "", str(e)
    except Exception as e:
        logger.exception("Gemini failed: %s", e)
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
        return RagResponse(
            ok=True,
            answer="No relevant text found.",
            hits=[],
        )

    hits_out = [_to_hitout(h) for h in filtered]

    # snippet-style answer
    parts = [f"[p.{h.page}, score={h.score:.3f}] {h.text}" for h in hits_out]
    answer = "\n\n---\n\n".join(parts)

    return RagResponse(
        ok=True,
        hits=hits_out,
        answer=answer,
    )


# ---------------- PDF RAG via Qdrant: clean final answer (Gemini optional) ----------------
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
        return RagResponse(
            ok=True,
            answer="No relevant text found.",
            hits=[],
        )

    hits_out = [_to_hitout(h) for h in filtered]
    context = _build_context(hits_out)

    # fallback always available
    fallback = "\n\n---\n\n".join([f"[p.{h.page}] {h.text}" for h in hits_out])[:2500]

    # try Gemini; never crash
    ans, llm_err = _gemini_summarize(q, context)
    if ans:
        return RagResponse(ok=True, hits=hits_out, answer=ans)
    else:
        return RagResponse(ok=True, hits=hits_out, answer=fallback, llm_error=llm_err)
