#!/usr/bin/env python3
"""
Debug script to check API key loading in the RAG pipeline
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

print("Environment variables check:")
print(f"GEMINI_API_KEY from os.getenv: {os.getenv('GEMINI_API_KEY')}")
print(f"GEMINI_GENERATION_MODEL from os.getenv: {os.getenv('GEMINI_GENERATION_MODEL')}")

# Now import rag_simple and check what it has
from rag_simple import GEMINI_API_KEY, GEMINI_GENERATION_MODEL
print(f"\nAfter importing rag_simple:")
print(f"GEMINI_API_KEY in rag_simple: {GEMINI_API_KEY}")
print(f"GEMINI_GENERATION_MODEL in rag_simple: {GEMINI_GENERATION_MODEL}")

# Test the initialization function
from rag_simple import initialize_services
qdrant_client, genai = initialize_services()
print(f"\nAfter initialize_services call:")
if genai:
    print("GenAI configured successfully")
else:
    print("GenAI configuration failed")