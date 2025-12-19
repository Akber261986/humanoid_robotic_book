import sys
import os
sys.path.insert(0, os.path.abspath('.'))  # Add current directory to path
sys.path.insert(0, os.path.abspath('..'))  # Add parent directory to path

# Set environment variables to load from the correct .env file
os.environ.setdefault('PYTHONPATH', '..')

from src.api.main import app
import uvicorn

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)