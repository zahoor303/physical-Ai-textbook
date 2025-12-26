from typing import List, Optional

import os
import logging

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models as rest

from .config import settings
from fastapi.middleware.cors import CORSMiddleware


# ---------------------------
# Logging
# ---------------------------

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(asctime)s - %(name)s - %(message)s",
)
logger = logging.getLogger("rag_backend")


# ---------------------------
# Gemini + Qdrant init
# ---------------------------

def init_gemini():
    genai.configure(api_key="AIzaSyA5jyZrJ0SSBflYFuuBjMA01mMxrBMs4mQ")
    # genai.configure(api_key=settings.gemini_api_key)
    logger.info("Gemini configured.")


def init_qdrant() -> QdrantClient:
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )
    logger.info("Connected to Qdrant at %s", settings.qdrant_url)
    return client


def embed_text(text: str) -> List[float]:
    if not text.strip():
        raise ValueError("Cannot embed empty text.")

    res = genai.embed_content(
        model=settings.model_embedding,
        content=text,
        task_type="retrieval_query",
    )
    return res["embedding"]


def generate_answer(prompt: str) -> str:
    model = genai.GenerativeModel(settings.model_chat)
    resp = model.generate_content(prompt)
    return resp.text or ""


qdrant = None
app = FastAPI(title="Physical AI Textbook RAG API")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",          # Docusaurus dev
        "https://your-book-domain.com",   # GitHub Pages / Vercel later
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.on_event("startup")
def startup_event():
    global qdrant
    init_gemini()
    qdrant = init_qdrant()
    logger.info("Startup complete.")


# ---------------------------
# Pydantic models
# ---------------------------

class ChatRequest(BaseModel):
    query: str
    top_k: int = 5
    chapter_slug: Optional[str] = None


class ChatResponse(BaseModel):
    answer: str
    contexts: List[dict]


class SelectionRequest(BaseModel):
    selection_text: str
    question: Optional[str] = None


class HealthResponse(BaseModel):
    status: str


# qdrant_client = QdrantClient(
#     url="https://901ae91b-e314-4c6d-82b3-0761f2b9f1a5.us-east-1-1.aws.cloud.qdrant.io:6333", 
#     api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Ef5UJqZdnNw4qTHBWn8J4w_a_hq0LfRRukK02eJ583g",
# )

# print(qdrant_client.get_collections())
# ---------------------------
# Helpers
# ---------------------------

def search_qdrant(
    query_embedding: List[float],
    top_k: int = 5,
    chapter_slug: Optional[str] = None,
):
    """Search Qdrant for the most relevant book chunks and return a list of ScoredPoint."""
    search_filter = None
    if chapter_slug:
        search_filter = rest.Filter(
            must=[
                rest.FieldCondition(
                    key="slug",
                    match=rest.MatchValue(value=chapter_slug),
                )
            ]
        )

    response = qdrant.query_points(
        collection_name=settings.qdrant_collection,
        query=query_embedding,
        limit=top_k,
        query_filter=search_filter,
        with_payload=True,
        with_vectors=False,
    )

    # Debug once to verify structure
    logger.info("Qdrant query_points raw response type=%s keys=%s",
                type(response), list(response.__dict__.keys()))

    return response.points


def build_rag_prompt(user_query: str, contexts: List[dict]) -> str:
    context_texts = []
    for c in contexts:
        context_texts.append(
            f"[{c.get('title', '')} - {c.get('heading', '')}]\n{c.get('text', '')}"
        )

    joined_context = "\n\n---\n\n".join(context_texts)

    prompt = f"""
You are a helpful tutor for a textbook about Physical AI & Humanoid Robotics.

Answer the user's question using ONLY the context below. 
If the answer is not in the context, say you don't see it in the book.

Context:
{joined_context}

Question:
{user_query}

Answer in clear, structured English suitable for a student.
"""
    return prompt.strip()


# ---------------------------
# Routes
# ---------------------------

@app.get("/health", response_model=HealthResponse)
def health():
    return HealthResponse(status="ok")


@app.post("/chat", response_model=ChatResponse)
def chat(req: ChatRequest):
    try:
        query_emb = embed_text(req.query)
    except Exception as e:
        logger.exception("Failed to embed query: %s", e)
        raise HTTPException(status_code=500, detail="Embedding failed")

    try:
        points = search_qdrant(
            query_emb,
            top_k=req.top_k,
            chapter_slug=req.chapter_slug,
        )
    except Exception as e:
        logger.exception("Qdrant search failed: %s", e)
        raise HTTPException(status_code=500, detail="Vector search failed")

    # logger.info("Retrieved %d points from Qdrant", len(points))

    contexts: List[dict] = []
    for p in points:
        payload = p.payload or {}
        contexts.append(
            {
                "text": payload.get("text", ""),
                "title": payload.get("title", ""),
                "slug": payload.get("slug", ""),
                "heading": payload.get("heading", ""),
                "score": p.score,
            }
        )

    if not contexts:
        answer = "I could not find anything in the textbook related to that question."
    else:
        prompt = build_rag_prompt(req.query, contexts)
        try:
            answer = generate_answer(prompt)
        except Exception as e:
            logger.exception("Gemini generation failed: %s", e)
            raise HTTPException(status_code=500, detail="Generation failed")

    return ChatResponse(answer=answer, contexts=contexts)

@app.post("/ask-on-selection", response_model=ChatResponse)
def ask_on_selection(req: SelectionRequest):
    """
    Answer based purely on user-selected text from the page.
    No vector search. This directly satisfies the hackathon requirement.
    """
    if not req.selection_text.strip():
        raise HTTPException(status_code=400, detail="selection_text cannot be empty")

    question = req.question or "Explain this text in simple terms."

    context = {
        "text": req.selection_text,
        "title": "User selection",
        "slug": "selection",
        "heading": "",
        "score": 1.0,
    }

    prompt = build_rag_prompt(question, [context])

    try:
        answer = generate_answer(prompt)
    except Exception as e:
        logger.exception("Gemini generation failed (selection): %s", e)
        raise HTTPException(status_code=500, detail="Generation failed")

    return ChatResponse(answer=answer, contexts=[context])


# What is physical AI?