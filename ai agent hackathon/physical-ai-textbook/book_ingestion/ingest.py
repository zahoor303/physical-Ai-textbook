#!/usr/bin/env python
"""
ingest.py  (starter version)

Goal:
- Initialize Gemini embeddings client
- Initialize Qdrant client
- Resolve the Docusaurus docs/ directory
- Provide a clean structure for later steps:
  - scan_markdown_files()
  - chunk_markdown_file()
  - embed_chunks_with_gemini()
  - upsert_chunks_to_qdrant()

You will run this with something like:
    uv run python ingest.py
"""

import os
import sys
import logging
from pathlib import Path
from typing import List, Dict, Any, Optional

import google.generativeai as genai  # Gemini embeddings SDK :contentReference[oaicite:0]{index=0}
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams

import re
import frontmatter
from markdown_it import MarkdownIt
import uuid


# ----------------------------
# Logging setup
# ----------------------------

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(asctime)s - %(name)s - %(message)s",
)

logger = logging.getLogger("book_ingestion")
md_parser = MarkdownIt()

# ----------------------------
# Configuration helpers
# ----------------------------

def get_env_var(name: str) -> str:
    """Read a required env var or crash with a clear error."""
    value = os.getenv(name)
    if not value:
        logger.error("Environment variable %s is not set.", name)
        raise RuntimeError(f"Environment variable {name} is required but not set.")
    return value


def resolve_docs_dir() -> Path:
    """
    Resolve the Docusaurus docs directory.

    Assumes this file lives in:
        PHYSICAL-AI-TEXTBOOK/book_ingestion/ingest.py

    So the docs folder is at:
        PHYSICAL-AI-TEXTBOOK/docs
    """
    project_root = Path(__file__).resolve().parents[1]
    docs_dir = project_root / "docs"

    if not docs_dir.exists():
        logger.error("Could not find docs directory at: %s", docs_dir)
        raise FileNotFoundError(f"docs directory not found at: {docs_dir}")

    logger.info("Using docs directory: %s", docs_dir)
    return docs_dir


# ----------------------------
# Gemini (embeddings) setup
# ----------------------------

def init_gemini() -> None:
    """
    Configure the Gemini embeddings client using GEMINI_API_KEY.

    Uses the 'text-embedding-004' model by default. :contentReference[oaicite:1]{index=1}
    """
    # api_key = get_env_var("GEMINI_API_KEY")
    api_key = "AIzaSyA5jyZrJ0SSBflYFuuBjMA01mMxrBMs4mQ"
    genai.configure(api_key=api_key)
    logger.info("Gemini client configured successfully.")


def embed_text_with_gemini(text: str, model: str = "models/text-embedding-004") -> List[float]:
    """
    Get a single embedding vector for a piece of text using Gemini.

    For now this is a simple one-off call; later we can add batching.
    """
    if not text.strip():
        raise ValueError("Cannot embed empty text.")

    logger.debug("Requesting embedding from Gemini (len=%d chars)", len(text))

    # google.generativeai.embed_content returns a dict with 'embedding' key :contentReference[oaicite:2]{index=2}
    result = genai.embed_content(
        model=model,
        content=text,
        task_type="retrieval_document",  # good default for RAG
    )

    embedding = result["embedding"]
    logger.debug("Received embedding of length %d", len(embedding))
    return embedding


# ----------------------------
# Qdrant setup
# ----------------------------

def init_qdrant_client() -> QdrantClient:
    """
    Initialize Qdrant client using QDRANT_URL and QDRANT_API_KEY.

    You can get these from your Qdrant Cloud dashboard:
    - URL:  e.g. https://YOUR-INSTANCE-xxx.region.azure.qdrant.io
    - API key: from 'API Keys' section.
    """
    url = "https://901ae91b-e314-4c6d-82b3-0761f2b9f1a5.us-east-1-1.aws.cloud.qdrant.io:6333"
    api_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Ef5UJqZdnNw4qTHBWn8J4w_a_hq0LfRRukK02eJ583g"

    client = QdrantClient(url=url, api_key=api_key)
    logger.info("Connected to Qdrant at %s", url)
    return client


def ensure_collection(
    client: QdrantClient,
    collection_name: str,
    vector_size: int,
    distance: Distance = Distance.COSINE,
) -> None:
    """
    Ensure a Qdrant collection exists with the correct vector size.

    We will call this AFTER generating the first embedding so we know vector_size.
    """
    from qdrant_client.http import models as rest

    existing_collections = [c.name for c in client.get_collections().collections]
    if collection_name in existing_collections:
        logger.info("Collection '%s' already exists in Qdrant.", collection_name)
        return

    logger.info(
        "Creating Qdrant collection '%s' with vector_size=%d, distance=%s",
        collection_name,
        vector_size,
        distance.value,
    )

    client.recreate_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=vector_size,
            distance=distance,
        ),
    )


# ----------------------------
# Placeholder structure for next steps
# ----------------------------

def scan_markdown_files(docs_dir: Path) -> List[Path]:
    """
    Walk the Docusaurus docs/ directory and collect all .md / .mdx files.

    Rules:
    - Include anything under docs/ ending with .md or .mdx
    - Skip obvious junk dirs if they exist (like node_modules inside docs/)
    """
    logger.info("Scanning for Markdown files in: %s", docs_dir)

    markdown_files: List[Path] = []

    # rglob will walk recursively under docs_dir
    for path in docs_dir.rglob("*"):
        if not path.is_file():
            continue

        # Skip unwanted directories if any (safety)
        parts = set(p.name for p in path.parents)
        if "node_modules" in parts or ".docusaurus" in parts:
            continue

        if path.suffix.lower() in {".md", ".mdx"}:
            markdown_files.append(path)

    markdown_files = sorted(markdown_files)

    logger.info("Found %d markdown files under docs/.", len(markdown_files))
    for p in markdown_files:
        logger.debug(" - %s", p.relative_to(docs_dir))

    return markdown_files




def build_all_chunks(docs_dir: Path) -> List[Dict[str, Any]]:
    """
    High-level helper:
    - Scan all markdown files
    - Parse each
    - Chunk each body
    - Attach metadata to each chunk

    Returns a flat list of chunk dicts:
      {
        "file_path": str,
        "slug": str,
        "title": str,
        "heading": str,
        "chunk_index": int,
        "text": str,
      }
    """
    markdown_files = scan_markdown_files(docs_dir)
    all_chunks: List[Dict[str, Any]] = []

    for path in markdown_files:
        parsed = parse_markdown_file(path)
        body = parsed["body_markdown"]
        chunks = chunk_markdown_body(body)

        for idx, ch in enumerate(chunks):
            all_chunks.append(
                {
                    "file_path": str(path),
                    "slug": parsed["slug"],
                    "title": parsed["title"],
                    "heading": ch["heading"],
                    "chunk_index": idx,
                    "text": ch["text"],
                }
            )

    logger.info("Total chunks built from all markdown files: %d", len(all_chunks))
    return all_chunks



def embed_chunks_with_gemini(
    chunks: List[Dict[str, Any]],
    model: str = "models/text-embedding-004",
) -> List[Dict[str, Any]]:
    """
    Take a list of text chunks and attach a Gemini embedding to each.

    Each chunk dict must have:
      - "text": str

    This function adds:
      - "embedding": List[float]
    """
    if not chunks:
        logger.warning("embed_chunks_with_gemini called with no chunks.")
        return chunks

    logger.info("Embedding %d chunks with Gemini...", len(chunks))

    for idx, chunk in enumerate(chunks):
        text = chunk.get("text", "").strip()
        if not text:
            logger.warning("Chunk %d has empty text. Skipping embedding.", idx)
            chunk["embedding"] = None
            continue

        try:
            embedding = embed_text_with_gemini(text, model=model)
            chunk["embedding"] = embedding
        except Exception as e:
            logger.exception("Failed to embed chunk %d: %s", idx, e)
            chunk["embedding"] = None

        if (idx + 1) % 20 == 0:
            logger.info("Embedded %d / %d chunks...", idx + 1, len(chunks))

    # Filter out chunks with no embedding
    embedded_chunks = [c for c in chunks if c.get("embedding") is not None]
    logger.info(
        "Successfully embedded %d/%d chunks.",
        len(embedded_chunks),
        len(chunks),
    )
    return embedded_chunks

def parse_markdown_file(path: Path) -> Dict[str, Any]:
    """
    Parse a single Docusaurus markdown/mdx file.

    - Reads frontmatter (title, id/slug, etc.)
    - Returns a dict with:
        {
          "path": Path,
          "title": str,
          "slug": str,
          "raw_markdown": str,
          "body_markdown": str,   # without frontmatter
        }
    """
    logger.info("Parsing markdown file: %s", path)

    # frontmatter.load can directly take a file path
    post = frontmatter.load(path)

    metadata = post.metadata or {}
    body_markdown = post.content or ""

    # Try to derive a reasonable title/slug
    title = metadata.get("title") or path.stem.replace("-", " ").title()
    slug = metadata.get("id") or metadata.get("slug") or path.stem

    return {
        "path": path,
        "title": title,
        "slug": slug,
        "metadata": metadata,
        "raw_markdown": post.content if hasattr(post, "content") else body_markdown,
        "body_markdown": body_markdown,
    }


def chunk_markdown_body(
    body_markdown: str,
    max_chars: int = 1200,
) -> List[Dict[str, Any]]:
    """
    Chunk a markdown body into smaller pieces for embeddings.

    Strategy:
    - Walk line by line
    - Start a new section when we hit a heading (#, ##, ###, ...)
    - For each section, if it's too long, split further by paragraphs
    - Returns a list of chunks:
        [
          { "heading": str, "text": str },
          ...
        ]
    """
    lines = body_markdown.splitlines()
    chunks: List[Dict[str, Any]] = []

    current_heading = "Introduction"
    current_buffer: List[str] = []

    heading_regex = re.compile(r"^(#{1,6})\s+(.*)")

    def flush_buffer():
        """Turn the current buffer into one or more chunks."""
        nonlocal current_buffer, current_heading, chunks

        if not current_buffer:
            return

        full_text = "\n".join(current_buffer).strip()
        if not full_text:
            current_buffer = []
            return

        # If the section is short enough, one chunk is fine
        if len(full_text) <= max_chars:
            chunks.append({"heading": current_heading, "text": full_text})
        else:
            # Split by double newlines into paragraphs and pack them
            paragraphs = full_text.split("\n\n")
            temp_buf: List[str] = []
            temp_len = 0

            for para in paragraphs:
                para = para.strip()
                if not para:
                    continue
                if temp_len + len(para) + 2 > max_chars and temp_buf:
                    # emit current packed chunk
                    chunk_text = "\n\n".join(temp_buf)
                    chunks.append({"heading": current_heading, "text": chunk_text})
                    temp_buf = [para]
                    temp_len = len(para)
                else:
                    temp_buf.append(para)
                    temp_len += len(para) + 2

            if temp_buf:
                chunk_text = "\n\n".join(temp_buf)
                chunks.append({"heading": current_heading, "text": chunk_text})

        current_buffer = []

    for line in lines:
        m = heading_regex.match(line.strip())
        if m:
            # New heading → flush previous section first
            flush_buffer()
            current_heading = m.group(2).strip() or current_heading
        else:
            current_buffer.append(line)

    # Flush whatever is left
    flush_buffer()

    logger.info("Created %d chunks from one markdown body.", len(chunks))
    return chunks




import uuid  # add at top of file if not already

def upsert_chunks_to_qdrant(
    client: QdrantClient,
    collection_name: str,
    chunks: List[Dict[str, Any]],
    batch_size: int = 64,
) -> None:
    """
    Upsert embedded chunks into Qdrant.

    Each chunk must have:
      - "embedding": List[float]
      - "text": str
      - "slug", "title", "heading", "file_path", "chunk_index" (metadata)

    We send:
      - id: uuid4
      - vector: embedding
      - payload: all useful metadata + text
    """
    if not chunks:
        logger.warning("upsert_chunks_to_qdrant called with no chunks.")
        return

    logger.info(
        "Upserting %d chunks into Qdrant collection '%s' (batch_size=%d)...",
        len(chunks),
        collection_name,
        batch_size,
    )

    from qdrant_client.http import models as rest

    # Simple batching
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i : i + batch_size]

        points = []
        for ch in batch:
            points.append(
                rest.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=ch["embedding"],
                    payload={
                        "text": ch["text"],
                        "slug": ch.get("slug"),
                        "title": ch.get("title"),
                        "heading": ch.get("heading"),
                        "file_path": ch.get("file_path"),
                        "chunk_index": ch.get("chunk_index"),
                    },
                )
            )

        try:
            client.upsert(
                collection_name=collection_name,
                points=points,
                wait=True,
            )
            logger.info(
                "Upserted batch %d–%d into Qdrant.",
                i + 1,
                i + len(batch),
            )
        except Exception as e:
            logger.exception(
                "Failed to upsert batch %d–%d into Qdrant: %s",
                i + 1,
                i + len(batch),
                e,
            )

    logger.info("Finished upserting all chunks into Qdrant.")



def main() -> None:
    """
    Full ingestion pipeline:

    - Initialize Gemini
    - Initialize Qdrant
    - Resolve docs/ directory
    - Scan + parse + chunk markdown
    - Embed chunks with Gemini
    - Ensure Qdrant collection exists
    - Upsert vectors into Qdrant
    """
    logger.info("Starting book ingestion (full pipeline).")

    # 1. Setup clients
    init_gemini()
    qdrant_client = init_qdrant_client()

    # 2. Resolve docs directory
    docs_dir = resolve_docs_dir()

    # 3. Build chunks from all markdown files
    all_chunks = build_all_chunks(docs_dir)
    logger.info("Ingestion pipeline produced %d chunks (pre-embedding).", len(all_chunks))

    if not all_chunks:
        logger.warning("No chunks produced from docs. Nothing to ingest.")
        return

    # 4. Embed chunks with Gemini
    embedded_chunks = embed_chunks_with_gemini(all_chunks)

    if not embedded_chunks:
        logger.error("No chunks were successfully embedded. Aborting.")
        return

    # 5. Ensure Qdrant collection exists
    vector_size = len(embedded_chunks[0]["embedding"])
    collection_name = "physical_ai_book"

    ensure_collection(
        client=qdrant_client,
        collection_name=collection_name,
        vector_size=vector_size,
    )

    qdrant_client.create_payload_index(
        collection_name=collection_name,
        field_name="slug",
        field_schema="keyword"
    )

    # 6. Upsert embeddings into Qdrant
    upsert_chunks_to_qdrant(
        client=qdrant_client,
        collection_name=collection_name,
        chunks=embedded_chunks,
    )

    logger.info("Book ingestion completed successfully.")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.exception("Ingestion script failed: %s", e)
        sys.exit(1)
