import sys
sys.path.append('./backend')

from backend.rag_simple import GEMINI_API_KEY, GEMINI_GENERATION_MODEL

print("Debug: Values loaded in rag_simple module")
print(f"GEMINI_API_KEY: {GEMINI_API_KEY}")
print(f"GEMINI_GENERATION_MODEL: {GEMINI_GENERATION_MODEL}")

# Also test loading from environment directly
import os
from dotenv import load_dotenv
load_dotenv()

print("\nDebug: Values loaded from .env directly")
print(f"Direct API key: {os.getenv('GEMINI_API_KEY')}")
print(f"Direct model: {os.getenv('GEMINI_GENERATION_MODEL')}")