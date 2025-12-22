#!/usr/bin/env python3
"""
Script to create a new Qdrant collection with the correct dimensions for TF-IDF embeddings
"""
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = "book_vectors_tfidf"

def recreate_collection():
    """Recreate the collection with correct dimensions for TF-IDF"""
    # Initialize Qdrant client
    if QDRANT_API_KEY:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    else:
        qdrant_client = QdrantClient(url=QDRANT_URL)
    
    print(f"Connecting to Qdrant at {QDRANT_URL}")
    
    # Try to delete existing collection
    try:
        qdrant_client.delete_collection(QDRANT_COLLECTION_NAME)
        print(f"Deleted existing collection: {QDRANT_COLLECTION_NAME}")
    except Exception as e:
        print(f"Collection {QDRANT_COLLECTION_NAME} may not exist yet: {e}")
    
    # Create new collection with 1000 dimensions for TF-IDF
    qdrant_client.create_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=models.VectorParams(size=1000, distance=models.Distance.COSINE),
    )
    
    print(f"Created new collection '{QDRANT_COLLECTION_NAME}' with 1000 dimensions for TF-IDF embeddings")
    
    # Verify collection was created
    collection_info = qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
    print(f"Collection info: {collection_info.config.params}")
    
    return qdrant_client

if __name__ == "__main__":
    client = recreate_collection()
    print("Collection is ready for TF-IDF embeddings!")