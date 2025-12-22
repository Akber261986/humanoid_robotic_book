#!/usr/bin/env python3
"""
Script to run the embedding process with proper environment loading
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Add the project root to the Python path so we can import modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Now import and run the embedding function
from process_book import embed_to_qdrant

if __name__ == "__main__":
    print("Starting embedding process with loaded environment variables...")
    result = embed_to_qdrant()
    print("Embedding process completed.")
    print(result)