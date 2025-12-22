#!/usr/bin/env python3
"""
Test the new API key with the exact model name from the list
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

        # Try with a known available model from the list
        print("Trying with gemini-pro-latest...")
        model = genai.GenerativeModel('models/gemini-pro-latest')
        response = model.generate_content("Hello, how are you?")
        print(f"Success! Response: {response.text[:100]}...")

    except Exception as e:
        print(f"Error with gemini-pro-latest: {e}")

        try:
            print("\nTrying with gemini-2.5-flash...")
            model = genai.GenerativeModel('models/gemini-2.5-flash')
            response = model.generate_content("Hello, how are you?")
            print(f"Success! Response: {response.text[:100]}...")
        except Exception as e2:
            print(f"Error with gemini-2.5-flash: {e2}")
else:
    print("No API key found in environment")