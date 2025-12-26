from __future__ import annotations

import re
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from rag import load_pdf_chunks, search_chunks

app = FastAPI(title="Hackathon RAG API", version="0.0.1")

# ---------------- CORS ----------------
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:8000",
        "http://127.0.0.1:8000",
        "https://mohammadfaisalpirzada.github.io",
    ],
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

    scored: list[tuple[int, str]] = []
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


# ---------------- PDF RAG (no LLM yet; returns best-matching chunks) ----------------
PDF_NAME = "Hackathon I- Physical AI & Humanoid Robotics Textbook.pdf"

# Usually repo root is 2 levels up from apps/api/main.py, but keep fallback.
PDF_PATH = Path(__file__).resolve().parents[2] / PDF_NAME
if not PDF_PATH.exists():
    PDF_PATH = Path(__file__).resolve().parents[3] / PDF_NAME

PDF_CHUNKS = []


@app.on_event("startup")
def _startup():
    global PDF_CHUNKS

    print("PDF_PATH =", PDF_PATH)
    print("PDF exists =", PDF_PATH.exists())

    if PDF_PATH.exists():
        PDF_CHUNKS = load_pdf_chunks(str(PDF_PATH))
        print("PDF chunks loaded =", len(PDF_CHUNKS))
    else:
        PDF_CHUNKS = []


@app.post("/pdf_chat")
def pdf_chat(req: ChatRequest):
    if not PDF_CHUNKS:
        return {
            "answer": (
                "PDF not loaded.\n"
                f"Expected at: {PDF_PATH}\n"
                "Put the PDF in repo root (same level as package.json) and restart API."
            )
        }

    hits = search_chunks(PDF_CHUNKS, req.message, k=3)
    if not hits:
        return {"answer": "No relevant text found in PDF for this question."}

    
    snippets = []
    sources = []
    for h in hits:
        page = h.meta.get("page")
        score = h.meta.get("score", 0.0)
        sources.append({"page": page, "score": score})
        snippets.append(h.text[:400])  # limit each chunk

    answer = "\n\n---\n\n".join([f"[p.{sources[i]['page']}, score={sources[i]['score']:.3f}] {snippets[i]}" for i in range(len(snippets))])
    return {"answer": answer, "sources": sources}       