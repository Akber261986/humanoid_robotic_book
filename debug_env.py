import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Check the values
print("GEMINI_API_KEY:", os.getenv("GEMINI_API_KEY", "NOT SET"))
print("GEMINI_GENERATION_MODEL:", os.getenv("GEMINI_GENERATION_MODEL", "NOT SET (using default)"))

# Check the .env file content
print("\nContent of .env file:")
with open('.env', 'r') as f:
    print(f.read())