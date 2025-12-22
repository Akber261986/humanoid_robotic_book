#!/usr/bin/env python3
"""
Script to embed book content directly to Qdrant cloud instance
Run this script once to populate your Qdrant collection with book content
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
import re

# Load environment variables
load_dotenv()

# Configuration - use Qdrant cloud settings
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_vectors")

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
        text = re.sub(r'\s+', ' ', text).strip()

        return text
    except Exception as e:
        print(f"Error reading markdown file {file_path}: {e}")
        return ""

def main():
    """Main function to embed book content to Qdrant cloud"""
    print("Starting to embed book content to Qdrant cloud...")
    print(f"Using QDRANT_URL: {QDRANT_URL}")
    print(f"Using collection: {QDRANT_COLLECTION_NAME}")

    # Initialize Qdrant client with cloud settings
    if QDRANT_API_KEY:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        print("Connected to Qdrant cloud with API key")
    else:
        qdrant_client = QdrantClient(url=QDRANT_URL)
        print("Connected to Qdrant cloud without API key")

    # Ensure collection exists
    try:
        qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
        print(f"Collection {QDRANT_COLLECTION_NAME} already exists")
        # Clear existing collection if needed
        confirm = input(f"Collection exists. Clear it first? (y/N): ")
        if confirm.lower() == 'y':
            qdrant_client.delete_collection(QDRANT_COLLECTION_NAME)
            print(f"Deleted existing collection {QDRANT_COLLECTION_NAME}")
        else:
            print("Using existing collection without clearing")
    except:
        # Create collection with 1000 dimensions for TF-IDF
        qdrant_client.create_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1000, distance=models.Distance.COSINE),
        )
        print(f"Created collection {QDRANT_COLLECTION_NAME}")

    # Find all markdown files in docs directory
    docs_path = "./docs"
    if not os.path.exists(docs_path):
        print(f"Docs directory does not exist: {docs_path}")
        return

    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    print(f"Found {len(markdown_files)} markdown files to process")

    if not markdown_files:
        print("No markdown files found in docs directory")
        return

    # Load the TF-IDF vectorizer that was saved during initial embedding
    try:
        vectorizer_path = "./tfidf_vectorizer.pkl"
        with open(vectorizer_path, 'rb') as f:
            vectorizer = pickle.load(f)
        print(f"Loaded TF-IDF vectorizer from {vectorizer_path}")
    except FileNotFoundError:
        print("TF-IDF vectorizer not found. Run embedding script first.")
        return

    total_chunks = 0
    # Process each markdown file
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
        for i, chunk in enumerate(chunks):
            if not chunk.strip():
                continue

            # Generate embedding using the loaded TF-IDF vectorizer
            try:
                chunk_embedding = vectorizer.transform([chunk]).toarray()[0].tolist()

                # Create Qdrant point
                point = models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=chunk_embedding,
                    payload={
                        "text": chunk,
                        "source": os.path.relpath(file_path, os.path.dirname(os.path.abspath(__file__))),
                        "chunk_id": i
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
                print(f"Error processing chunk {i} in {file_path}: {e}")
                continue

    print(f"Successfully embedded {total_chunks} chunks to Qdrant cloud collection '{QDRANT_COLLECTION_NAME}'")

if __name__ == "__main__":
    main()