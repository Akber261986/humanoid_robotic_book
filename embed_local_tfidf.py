#!/usr/bin/env python3
"""
Direct TF-IDF embedding script to use local in-memory Qdrant
"""
import os
import glob
import uuid
import pickle
from typing import List
import markdown
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
from sklearn.feature_extraction.text import TfidfVectorizer
import numpy as np

# Load environment variables but override URL for local usage
load_dotenv()

# Configuration - use local in-memory mode
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_vectors")

def initialize_services():
    """Initialize Qdrant client in local/mem mode"""
    # Use in-memory mode for local embedding
    qdrant_client = QdrantClient(":memory:")
    print("Qdrant client initialized in memory mode")
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

def embed_text_with_tfidf(text: str, vectorizer) -> List[float]:
    """Generate embedding using TF-IDF vectorizer"""
    try:
        # Transform the text
        embedding = vectorizer.transform([text])

        # Convert sparse matrix to dense and then to list
        embedding_dense = embedding.toarray()[0]
        embedding_list = embedding_dense.tolist()

        return embedding_list
    except Exception as e:
        print(f"Error generating TF-IDF embedding: {e}")
        return []

def ensure_collection_exists(client: QdrantClient, collection_name: str):
    """Ensure the Qdrant collection exists with correct dimensions"""
    try:
        client.get_collection(collection_name)
        print(f"Collection {collection_name} already exists")
    except:
        # Create collection with 1000 dimensions for TF-IDF
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1000, distance=models.Distance.COSINE),
        )
        print(f"Created collection {collection_name}")

def embed_book_to_qdrant():
    """Embed all book content to Qdrant using TF-IDF"""
    print("Starting to embed book content to Qdrant using TF-IDF (local in-memory)...")
    print(f"Using collection: {QDRANT_COLLECTION_NAME}")
    print("Using local in-memory Qdrant storage...")

    # Initialize services
    qdrant_client = initialize_services()

    # Ensure collection exists
    ensure_collection_exists(qdrant_client, QDRANT_COLLECTION_NAME)

    # Find all markdown files
    docs_path = "./docs"
    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)

    print(f"Found {len(markdown_files)} markdown files to process")

    # First, collect all texts to fit the vectorizer
    all_texts = []
    file_chunks_map = {}  # To keep track of which chunks belong to which files

    for file_path in markdown_files:
        print(f"Preprocessing file: {file_path}")

        # Extract text from markdown
        text_content = extract_text_from_md(file_path)

        if not text_content.strip():
            print(f"No content extracted from {file_path}")
            continue

        # Split into chunks
        chunks = chunk_text(text_content)

        # Store chunks for vectorizer fitting and mapping
        for i, chunk in enumerate(chunks):
            if chunk.strip():
                all_texts.append(chunk)
                file_chunks_map[len(all_texts)-1] = (file_path, i)  # Store file and chunk index

    print(f"Collected {len(all_texts)} text chunks for vectorizer fitting")

    if not all_texts:
        print("No text content to process")
        return {"status": "failed", "chunks_embedded": 0, "files_processed": 0}

    # Initialize and fit the vectorizer on all texts
    print("Fitting TF-IDF vectorizer on all content...")
    vectorizer = TfidfVectorizer(
        max_features=1000,  # Limit to 1000 features to match collection dimensions
        stop_words='english',
        lowercase=True,
        ngram_range=(1, 2)  # Use both unigrams and bigrams
    )

    # Fit the vectorizer on all texts
    vectorizer.fit(all_texts)
    print(f"TF-IDF vectorizer fitted with vocabulary size: {len(vectorizer.vocabulary_)}")

    # Now process each chunk and embed it
    total_chunks = 0
    for idx, text in enumerate(all_texts):
        file_path, chunk_idx = file_chunks_map[idx]

        # Generate embedding using TF-IDF
        try:
            embedding = embed_text_with_tfidf(text, vectorizer)

            if not embedding or len(embedding) != 1000:
                print(f"Failed to generate valid embedding for chunk {chunk_idx} in {file_path}")
                continue

            # Create Qdrant point
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "text": text,
                    "source": file_path,
                    "chunk_id": chunk_idx
                }
            )

            # Upload single point to Qdrant
            qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=[point]
            )

            total_chunks += 1

            if total_chunks % 50 == 0:  # Print progress every 50 chunks
                print(f"Embedded {total_chunks} chunks so far...")

        except Exception as e:
            print(f"Error processing chunk {chunk_idx} in {file_path}: {e}")
            continue

    print(f"Successfully embedded {total_chunks} chunks to Qdrant collection '{QDRANT_COLLECTION_NAME}' in memory")
    print("Note: Data is stored in memory and will be lost when the program exits.")
    print("To persist data, run a local Qdrant server with 'docker run -d -p 6333:6333 qdrant/qdrant'")

    # Save the fitted vectorizer for later use in querying
    vectorizer_path = "./tfidf_vectorizer.pkl"
    with open(vectorizer_path, 'wb') as f:
        pickle.dump(vectorizer, f)
    print(f"Saved fitted vectorizer to {vectorizer_path}")

    return {"status": "completed", "chunks_embedded": total_chunks, "files_processed": len(markdown_files)}

if __name__ == "__main__":
    embed_book_to_qdrant()