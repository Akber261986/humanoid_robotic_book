#!/usr/bin/env python3
"""
Test script to verify the TF-IDF based RAG system works with the embedded book content
"""
import pickle
import glob
from typing import List, Dict, Any
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
from qdrant_client import QdrantClient
from qdrant_client.http import models
import sys
import os

def load_tfidf_rag_system():
    """Load the TF-IDF vectorizer and Qdrant client for testing"""
    print("Loading TF-IDF RAG System...")

    # Load the fitted vectorizer
    try:
        with open('./tfidf_vectorizer.pkl', 'rb') as f:
            vectorizer = pickle.load(f)
        print("Loaded TF-IDF vectorizer successfully")
    except FileNotFoundError:
        print("TF-IDF vectorizer not found. Please run the embedding script first.")
        return None, None

    # Initialize in-memory Qdrant client (this will have our embedded content from the session)
    try:
        # Create a new in-memory client and re-embed for this test
        print("Note: Since data is in memory, we need to re-embed for this session")
        qdrant_client = QdrantClient(":memory:")

        # Create collection with 1000 dimensions for TF-IDF
        qdrant_client.create_collection(
            collection_name="book_vectors",
            vectors_config=models.VectorParams(size=1000, distance=models.Distance.COSINE),
        )
        print("Created in-memory Qdrant collection")

        # Re-embed the content using the same approach as before
        print("Re-embedding content for this session...")
        reembed_content(qdrant_client, vectorizer)

    except Exception as e:
        print(f"✗ Error initializing Qdrant: {e}")
        return None, None

    return vectorizer, qdrant_client

def reembed_content(qdrant_client, vectorizer):
    """Re-embed content to the in-memory Qdrant for this test session"""
    import uuid
    import markdown

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

    # Find all markdown files
    docs_path = "./docs"
    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)

    print(f"Found {len(markdown_files)} markdown files to re-embed")

    # First, collect all texts to fit the vectorizer (in case we need to recreate it)
    all_texts = []
    file_chunks_map = {}  # To keep track of which chunks belong to which files

    for file_path in markdown_files:
        print(f"  Processing file: {file_path}")

        # Extract text from markdown
        text_content = extract_text_from_md(file_path)

        if not text_content.strip():
            print(f"  No content extracted from {file_path}")
            continue

        # Split into chunks
        chunks = chunk_text(text_content)

        # Store chunks for processing
        for i, chunk in enumerate(chunks):
            if chunk.strip():
                all_texts.append(chunk)
                file_chunks_map[len(all_texts)-1] = (file_path, i)  # Store file and chunk index

    print(f"  Collected {len(all_texts)} text chunks for re-embedding")

    # Now process each chunk and embed it
    total_chunks = 0
    for idx, text in enumerate(all_texts):
        file_path, chunk_idx = file_chunks_map[idx]

        # Generate embedding using TF-IDF
        try:
            embedding = embed_text_with_tfidf(text, vectorizer)

            if not embedding or len(embedding) != 1000:
                print(f"  Failed to generate valid embedding for chunk {chunk_idx} in {file_path}")
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
                collection_name="book_vectors",
                points=[point]
            )

            total_chunks += 1

            if total_chunks % 50 == 0:  # Print progress every 50 chunks
                print(f"  Re-embedded {total_chunks} chunks so far...")

        except Exception as e:
            print(f"  Error processing chunk {chunk_idx} in {file_path}: {e}")
            continue

    print(f"  Successfully re-embedded {total_chunks} chunks to Qdrant collection 'book_vectors'")

def query_tfidf_rag(query: str, vectorizer: TfidfVectorizer, qdrant_client: QdrantClient, top_k: int = 3) -> Dict[str, Any]:
    """Query the TF-IDF based RAG system"""
    try:
        # Generate embedding for the query using the same vectorizer
        query_embedding = vectorizer.transform([query])
        query_embedding_dense = query_embedding.toarray()[0].tolist()

        print(f"Query embedding generated with {len(query_embedding_dense)} dimensions")

        # Search in Qdrant collection
        search_results = qdrant_client.search(
            collection_name="book_vectors",
            query_vector=query_embedding_dense,
            limit=top_k,
            score_threshold=0.01  # Minimum similarity threshold
        )

        results = []
        for result in search_results:
            results.append({
                "text": result.payload.get("text", ""),
                "source": result.payload.get("source", ""),
                "chunk_id": result.payload.get("chunk_id", ""),
                "score": result.score
            })

        return {
            "query": query,
            "retrieved_docs": results,
            "retrieved_docs_count": len(results)
        }
    except Exception as e:
        print(f"Error querying TF-IDF RAG: {e}")
        import traceback
        print(f"Full traceback: {traceback.format_exc()}")
        return {
            "query": query,
            "retrieved_docs": [],
            "retrieved_docs_count": 0
        }

def test_tfidf_rag_system():
    """Test the TF-IDF RAG system with sample queries about the book content"""
    print("Testing TF-IDF RAG System with Embedded Book Content")
    print("=" * 60)

    # Load the system
    vectorizer, qdrant_client = load_tfidf_rag_system()

    if vectorizer is None or qdrant_client is None:
        print("Failed to load TF-IDF RAG system")
        return

    # Test queries related to humanoid robotics
    test_queries = [
        "What are the fundamentals of humanoid robotics?",
        "Explain inverse kinematics in humanoid robots",
        "What is ROS2 and how is it used in robotics?",
        "How does Gazebo work for robotics simulation?",
        "What are the key concepts in physical AI?",
        "Explain Isaac Sim for robotics simulation"
    ]

    print(f"\nRunning {len(test_queries)} test queries...")
    print()

    for i, query in enumerate(test_queries, 1):
        print(f"Query {i}: {query}")
        print("-" * 40)

        try:
            result = query_tfidf_rag(query, vectorizer, qdrant_client)

            print(f"Retrieved documents: {result['retrieved_docs_count']}")

            if result['retrieved_docs']:
                print("Retrieved content:")
                for j, doc in enumerate(result['retrieved_docs'], 1):
                    print(f"  {j}. Source: {doc['source']}")
                    print(f"     Score: {doc['score']:.4f}")
                    print(f"     Content preview: {doc['text'][:150]}...")
                    print()
            else:
                print("No relevant content retrieved")

        except Exception as e:
            print(f"Error processing query: {e}")

        print("="*60)
        print()

if __name__ == "__main__":
    test_tfidf_rag_system()