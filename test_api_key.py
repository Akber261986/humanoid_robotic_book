#!/usr/bin/env python3
"""
Test script to verify if the Gemini API key is valid
"""
import os
import sys
from dotenv import load_dotenv
import google.generativeai as genai

# Load environment variables from .env file
load_dotenv()

# Get the API key
api_key = os.getenv("GEMINI_API_KEY")
embedding_model = os.getenv("GEMINI_EMBEDDING_MODEL", "models/embedding-001")

print(f"API Key loaded: {'Yes' if api_key else 'No'}")
print(f"API Key (first 10 chars): {api_key[:10] if api_key else 'N/A'}")
print(f"Full API Key: {api_key}")
print(f"Embedding Model: {embedding_model}")

if api_key and api_key != "your_google_gemini_api_key_here":
    try:
        # Configure the API
        genai.configure(api_key=api_key)
        print("SUCCESS: API configured successfully")

        # Test embedding
        test_text = "This is a test for API validation"
        response = genai.embed_content(
            model=embedding_model,
            content=[test_text],
            task_type="retrieval_document"
        )

        print(f"SUCCESS: Embedding successful! Response type: {type(response)}")
        print(f"SUCCESS: Embedding length: {len(response['embedding'][0]) if 'embedding' in response and len(response['embedding']) > 0 else 0}")

    except Exception as e:
        print(f"ERROR: Error during API test: {e}")
        import traceback
        traceback.print_exc()
else:
    print("ERROR: No valid API key found or placeholder value detected")