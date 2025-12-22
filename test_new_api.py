#!/usr/bin/env python3
"""
Test the new API key directly
"""
import os
from dotenv import load_dotenv
import google.generativeai as genai

# Load environment variables
load_dotenv()

# Get the API key from environment
api_key = os.getenv("GEMINI_API_KEY")
model_name = os.getenv("GEMINI_GENERATION_MODEL", "gemini-1.5-pro-latest")

print(f"Using API key: {api_key}")
print(f"Using model: {model_name}")

if api_key:
    try:
        # Configure the API
        genai.configure(api_key=api_key)

        # Try to list models as a basic test
        print("Testing API connection...")
        models = genai.list_models()
        print(f"Available models: {len(list(models))}")

        # Try to create a simple model instance
        model = genai.GenerativeModel(model_name)
        print(f"Successfully created model instance for: {model_name}")

        # Try a simple generation
        response = model.generate_content("Hello, how are you?")
        print(f"Test response: {response.text[:100]}...")

    except Exception as e:
        print(f"Error testing API: {e}")
else:
    print("No API key found in environment")