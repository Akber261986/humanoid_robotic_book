"""
Script to embed book content to Qdrant vector database
"""
import os
import glob
import uuid
from typing import List, Dict, Any
import markdown
from qdrant_client import QdrantClient
from qdrant_client.http import models
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_EMBEDDING_MODEL = os.getenv("GEMINI_EMBEDDING_MODEL", "embedding-001")

def initialize_services():
    """Initialize Qdrant and Gemini services"""
    # Initialize Qdrant client
    if QDRANT_API_KEY:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        qdrant_client = QdrantClient(url=QDRANT_URL)

    # Initialize Gemini
    if GEMINI_API_KEY:
        genai.configure(api_key=GEMINI_API_KEY)

    return qdrant_client

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """Split text into overlapping chunks"""
    sentences = text.split('. ')
    chunks = []
    current_chunk = ""

    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        if len(current_chunk + sentence) < chunk_size:
            current_chunk += sentence + ". "
        else:
            if current_chunk:
                chunks.append(current_chunk.strip())

            # Create overlapping chunk
            words = current_chunk.split()
            overlap_start = max(0, len(words) - overlap)
            current_chunk = " ".join(words[overlap_start:]) + " " + sentence + ". "

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks

def extract_text_from_md(file_path: str) -> str:
    """Extract text content from a markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Convert markdown to plain text
        html = markdown.markdown(content)
        # Simple approach to extract text from HTML
        text = html.replace('<p>', ' ').replace('</p>', ' ').replace('<h1>', ' ').replace('</h1>', ' ') \
                  .replace('<h2>', ' ').replace('</h2>', ' ').replace('<h3>', ' ').replace('</h3>', ' ') \
                  .replace('<li>', ' ').replace('</li>', ' ').replace('<ul>', ' ').replace('</ul>', ' ') \
                  .replace('<strong>', ' ').replace('</strong>', ' ').replace('<em>', ' ').replace('</em>', ' ')

        # Remove extra whitespace
        import re
        text = re.sub(r'\s+', ' ', text).strip()

        return text
    except Exception as e:
        print(f"Error reading markdown file {file_path}: {e}")
        return ""

def embed_text(text: str) -> List[float]:
    """Generate embedding for text using Gemini"""
    try:
        response = genai.embed_content(
            model=GEMINI_EMBEDDING_MODEL,
            content=[text],
            task_type="retrieval_document"
        )
        return response['embedding'][0]
    except Exception as e:
        print(f"Error generating embedding: {e}")
        return []

def ensure_collection_exists(client: QdrantClient, collection_name: str):
    """Ensure the Qdrant collection exists"""
    try:
        client.get_collection(collection_name)
        print(f"Collection {collection_name} already exists")
    except:
        # Create collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
        )
        print(f"Created collection {collection_name}")

def embed_book_to_qdrant():
    """Embed all book content to Qdrant"""
    print("Starting to embed book content to Qdrant...")

    # Initialize services
    qdrant_client = initialize_services()

    # Ensure collection exists
    ensure_collection_exists(qdrant_client, QDRANT_COLLECTION_NAME)

    # Find all markdown files
    docs_path = "./docs"
    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)

    print(f"Found {len(markdown_files)} markdown files to process")

    total_chunks = 0

    for file_path in markdown_files:
        print(f"Processing file: {file_path}")

        # Extract text from markdown
        text_content = extract_text_from_md(file_path)

        if not text_content.strip():
            print(f"No content extracted from {file_path}")
            continue

        # Split into chunks
        chunks = chunk_text(text_content)

        # Process each chunk
        points = []
        for i, chunk in enumerate(chunks):
            if not chunk.strip():
                continue

            # Generate embedding
            try:
                embedding = embed_text(chunk)

                if not embedding:
                    print(f"Failed to generate embedding for chunk {i} in {file_path}")
                    continue

                # Create Qdrant point
                point = models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload={
                        "text": chunk,
                        "source": file_path,
                        "chunk_id": i
                    }
                )
                points.append(point)

            except Exception as e:
                print(f"Error processing chunk {i} in {file_path}: {e}")
                continue

        # Upload to Qdrant
        if points:
            qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=points
            )

            total_chunks += len(points)
            print(f"Embedded {len(points)} chunks from {file_path}")

    print(f"Successfully embedded {total_chunks} chunks to Qdrant collection '{QDRANT_COLLECTION_NAME}'")
    return {"status": "completed", "chunks_embedded": total_chunks, "files_processed": len(markdown_files)}

if __name__ == "__main__":
    embed_book_to_qdrant()