#!/usr/bin/env python3
"""
Script to run the embedding process with the correct API key set programmatically
"""
import os
import sys

# Set the correct API key programmatically before importing other modules
os.environ['GEMINI_API_KEY'] = 'AIzaSyBdTT8lecTofR9rTdGPZv6B61YLtG9o9bg'
os.environ['GEMINI_EMBEDDING_MODEL'] = 'models/embedding-001'
os.environ['QDRANT_COLLECTION_NAME'] = 'book_vectors'

# Add the project root to the Python path so we can import modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Now import and run the embedding function
from process_book import embed_to_qdrant

if __name__ == "__main__":
    print("Starting embedding process with programmatically set API key...")
    result = embed_to_qdrant()
    print("Embedding process completed.")
    print(result)