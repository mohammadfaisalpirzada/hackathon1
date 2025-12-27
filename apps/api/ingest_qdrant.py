from __future__ import annotations

import os

from pathlib import Path
from dotenv import load_dotenv
load_dotenv(Path(__file__).resolve().with_name(".env"))



from rag import load_pdf_chunks
from embed import embed_texts
from qdrant_store import ensure_collection, upsert_chunks

load_dotenv()

PDF_PATH = os.getenv("PDF_PATH", "")  # optional
PDF_NAME = os.getenv("PDF_NAME", "Hackathon I- Physical AI & Humanoid Robotics Textbook.pdf")
COLLECTION = os.getenv("QDRANT_COLLECTION", "hackathon1_book")


def main():
    # Use same logic as your main.py: run this from repo root or set PDF_PATH
    pdf_path = PDF_PATH or PDF_NAME

    chunks = load_pdf_chunks(pdf_path, fix_spaced=True)
    print("chunks:", len(chunks))
    if not chunks:
        raise SystemExit("No chunks extracted. PDF text extraction failed.")

    texts = [c.text for c in chunks]
    vectors = embed_texts(texts)
    print("vectors:", len(vectors), "dim:", len(vectors[0]))

    ensure_collection(COLLECTION, vector_size=len(vectors[0]))
    upsert_chunks(COLLECTION, vectors, chunks)
    print("✅ Ingested into Qdrant:", COLLECTION)


if __name__ == "__main__":
    main()