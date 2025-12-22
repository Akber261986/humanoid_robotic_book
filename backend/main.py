from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import os
import subprocess
import json
from typing import Dict, Any

# Import the RAG module
import sys
import os
# Add the backend directory to Python path to ensure rag module can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Use the fixed rag module that bypasses TF-IDF issues
from rag_simple_fixed import query_rag_pipeline, check_services

app = FastAPI(title="Humanoid Robotics Book API", version="1.0.0")

# Add CORS middleware to allow requests from any origin
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for the chatbot to work from any frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check endpoint
@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "Humanoid Robotics Book API"}

# Service status check
@app.get("/status")
def service_status():
    return check_services()

# Serve the HTML file
@app.get("/", response_class=HTMLResponse)
def read_root(request: Request):
    with open("frontend_chat.html", "r", encoding="utf-8") as file:
        content = file.read()
    return HTMLResponse(content=content)

@app.get("/chat")
def chat_page(request: Request):
    with open("frontend_chat.html", "r", encoding="utf-8") as file:
        content = file.read()
    return HTMLResponse(content=content)

@app.post("/trigger-index")
def trigger_index():
    """Trigger the script to process book markdown files (simple processing)"""
    try:
        # Run simple processing
        result = subprocess.run(
            ["python", "./process_book.py"],
            capture_output=True,
            text=True
        )

        return {
            "status": "success" if result.returncode == 0 else "error",
            "stdout": result.stdout,
            "stderr": result.stderr,
            "return_code": result.returncode
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.post("/embed-book")
def embed_book():
    """Trigger the script to embed book content to Qdrant"""
    try:
        # Run embedding to Qdrant
        result = subprocess.run(
            ["python", "./process_book.py", "embed"],
            capture_output=True,
            text=True
        )

        return {
            "status": "success" if result.returncode == 0 else "error",
            "stdout": result.stdout,
            "stderr": result.stderr,
            "return_code": result.returncode
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/api/query-book")
def query_book_api(query: str = None):
    """Query about the book content using RAG pipeline"""
    if not query:
        return {"error": "Please provide a query parameter"}

    try:
        # Use the RAG pipeline to get response from LLM with Qdrant integration
        result = query_rag_pipeline(query)
        return result
    except Exception as e:
        return {"error": str(e)}

# Keep the old query-book endpoint for backward compatibility
@app.get("/query-book")
def query_book(query: str = None):
    """Query about the book content (legacy endpoint)"""
    if not query:
        return {"error": "Please provide a query parameter"}

    try:
        # Use the RAG pipeline to get response from LLM with Qdrant integration
        result = query_rag_pipeline(query)
        return result
    except Exception as e:
        return {"error": str(e)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)