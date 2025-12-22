#!/usr/bin/env python3
"""
Test script to verify if the Gemini API key is valid by reading .env directly
"""
import os
import sys
from dotenv import load_dotenv

# Explicitly load the .env file from the current directory
env_path = os.path.join(os.path.dirname(__file__), '.env')
print(f"Loading .env file from: {env_path}")
print(f"File exists: {os.path.exists(env_path)}")

# Load environment variables
load_dotenv(dotenv_path=env_path)

# Get the API key
api_key = os.getenv("GEMINI_API_KEY")
embedding_model = os.getenv("GEMINI_EMBEDDING_MODEL", "models/embedding-001")

print(f"API Key loaded: {'Yes' if api_key else 'No'}")
print(f"API Key (first 10 chars): {api_key[:10] if api_key else 'N/A'}")
print(f"Full API Key: {'[REDACTED]' if api_key and api_key != 'your_google_gemini_api_key_here' else api_key}")
print(f"Embedding Model: {embedding_model}")

# Also try reading the file directly to confirm content
print("\n--- Direct file content ---")
with open(env_path, 'r') as f:
    content = f.read()
    print(content)

if api_key and api_key != "your_google_gemini_api_key_here":
    import google.generativeai as genai
    try:
        # Configure the API
        genai.configure(api_key=api_key)
        print("\nSUCCESS: API configured successfully")
        
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
    print("\nERROR: No valid API key found or placeholder value detected")
    print("Make sure to update your .env file with a valid Gemini API key")