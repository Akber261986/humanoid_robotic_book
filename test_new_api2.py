#!/usr/bin/env python3
"""
Test the new API key with a valid model
"""
import os
from dotenv import load_dotenv
import google.generativeai as genai

# Load environment variables
load_dotenv()

# Get the API key from environment
api_key = os.getenv("GEMINI_API_KEY")

if api_key:
    try:
        # Configure the API
        genai.configure(api_key=api_key)

        # List available models to see what's available
        print("Listing available models...")
        models = genai.list_models()
        for model in models:
            if 'gemini' in model.name.lower():
                print(f"  - {model.name}")

        # Try with a known available model
        print("\nTrying with gemini-pro...")
        model = genai.GenerativeModel('gemini-pro')
        response = model.generate_content("Hello, how are you?")
        print(f"Success! Response: {response.text[:100]}...")

    except Exception as e:
        print(f"Error: {e}")
else:
    print("No API key found in environment")