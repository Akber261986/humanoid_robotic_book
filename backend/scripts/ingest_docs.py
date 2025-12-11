#!/usr/bin/env python3
"""
Ingestion script for Physical AI & Humanoid Robotics book content into Qdrant vector database.
This script reads all Markdown files from the docs directory, chunks them, embeds with a local model,
and uploads to Qdrant.
"""
import os
import glob
import markdown
from pathlib import Path
from typing import List, Dict, Any
import hashlib
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastembed import TextEmbedding
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize local embedding model (efficient and lightweight) - will be set to None initially for potential lazy loading
embedding_model = None

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")  # Update with your Qdrant cluster URL
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_book")

def read_markdown_files(docs_path: str) -> List[Dict[str, Any]]:
    """Read all markdown files from the docs directory and extract content with metadata."""
    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    documents = []

    for file_path in markdown_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract metadata from file path
        relative_path = os.path.relpath(file_path, docs_path)
        doc_id = hashlib.md5(content.encode()).hexdigest()

        documents.append({
            'id': doc_id,
            'content': content,
            'file_path': relative_path,
            'source': f"docs/{relative_path}"
        })

    return documents

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """Split text into overlapping chunks."""
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)

        start += chunk_size - overlap
        if start >= len(text):
            break

    return chunks

def get_embedding(text: str) -> List[float]:
    """Get embedding for text using fastembed local model."""
    global embedding_model

    try:
        # Initialize the embedding model if not already loaded
        if embedding_model is None:
            print("Initializing embedding model...")
            embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
            print("Embedding model initialized successfully")

        # The fastembed model returns a generator, so we need to get the first result
        embedding = list(embedding_model.embed([text]))[0]
        return embedding.tolist()  # Convert to list
    except Exception as e:
        print(f"Error getting embedding for text: {str(e)}")
        return []

def create_qdrant_collection(client: QdrantClient, collection_name: str):
    """Create or recreate the Qdrant collection."""
    # Check if collection exists
    collections = client.get_collections().collections
    collection_names = [col.name for col in collections]

    if collection_name in collection_names:
        print(f"Collection '{collection_name}' already exists. Recreating...")
        client.delete_collection(collection_name)

    # Create new collection with appropriate vector size for BGE embeddings
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=384,  # BGE small model returns 384-dimensional vectors
            distance=models.Distance.COSINE
        )
    )
    print(f"Collection '{collection_name}' created successfully.")

def ingest_documents(docs_path: str, qdrant_client: QdrantClient, collection_name: str):
    """Ingest documents into Qdrant."""
    print(f"Reading documents from {docs_path}...")
    documents = read_markdown_files(docs_path)
    print(f"Found {len(documents)} documents")

    points = []
    processed_chunks = 0

    for doc in documents:
        print(f"Processing {doc['source']}...")

        # Chunk the document content
        chunks = chunk_text(doc['content'])

        for i, chunk in enumerate(chunks):
            # Generate embedding for the chunk
            embedding = get_embedding(chunk)
            if not embedding:
                continue

            # Create a unique ID for this chunk
            chunk_id = f"{doc['id']}_chunk_{i}"

            # Create the point for Qdrant
            point = models.PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={
                    "content": chunk,
                    "source": doc['source'],
                    "file_path": doc['file_path'],
                    "chunk_index": i
                }
            )

            points.append(point)
            processed_chunks += 1

            # Batch upload every 100 points to avoid memory issues
            if len(points) >= 100:
                qdrant_client.upsert(
                    collection_name=collection_name,
                    points=points
                )
                print(f"Uploaded batch of {len(points)} chunks...")
                points = []

    # Upload remaining points
    if points:
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        print(f"Uploaded final batch of {len(points)} chunks...")

    print(f"Successfully ingested {processed_chunks} chunks into Qdrant collection '{collection_name}'")

def main():
    """Main function to run the ingestion process."""
    docs_path = "docs"

    # Initialize Qdrant client
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        prefer_grpc=True  # Use gRPC for better performance
    )

    # Create or recreate the collection
    create_qdrant_collection(client, QDRANT_COLLECTION_NAME)

    # Ingest documents
    ingest_documents(docs_path, client, QDRANT_COLLECTION_NAME)

    print("Ingestion completed successfully!")

if __name__ == "__main__":
    main()