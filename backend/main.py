from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
import os
import subprocess
import json
from typing import Dict, Any

app = FastAPI(title="Humanoid Robotics Book API", version="1.0.0")

# Add CORS middleware to allow requests from GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://Akber261986.github.io", "http://localhost:3000", "http://localhost:3001", "http://localhost:8080"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check endpoint
@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "Humanoid Robotics Book API"}

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
            ["python", "/app/process_book.py"],
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
            ["python", "/app/process_book.py", "embed"],
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

@app.get("/query-book")
def query_book(query: str = None):
    """Query about the book content"""
    if not query:
        return {"error": "Please provide a query parameter"}

    # Simple keyword matching in markdown files
    try:
        results = []
        docs_path = "/app/docs"

        for root, dirs, files in os.walk(docs_path):
            for file in files:
                if file.endswith('.md'):
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()

                        # Simple search - look for query in content
                        if query.lower() in content.lower():
                            # Extract context around the query
                            lines = content.split('\n')
                            matching_lines = []

                            for i, line in enumerate(lines):
                                if query.lower() in line.lower():
                                    # Get context: previous line, matching line, next line
                                    start = max(0, i-1)
                                    end = min(len(lines), i+2)
                                    context = '\n'.join(lines[start:end])
                                    matching_lines.append({
                                        "file": file,
                                        "context": context
                                    })

                            if matching_lines:
                                results.extend(matching_lines)

        return {
            "query": query,
            "results": results[:5],  # Limit to first 5 results
            "total_matches": len(results)
        }
    except Exception as e:
        return {"error": str(e)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)