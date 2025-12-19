from fastapi import FastAPI
import os
import subprocess
import json
from typing import Dict, Any

app = FastAPI(title="Humanoid Robotics Book API", version="1.0.0")

@app.get("/")
def read_root():
    return {"message": "Humanoid Robotics Book API", "endpoints": ["/trigger-index", "/embed-book", "/query-book"]}

@app.post("/trigger-index")
def trigger_index():
    """Trigger the script to process book markdown files (simple processing)"""
    try:
        # Run simple processing
        result = subprocess.run(
            ["python", "../process_book.py"],
            capture_output=True,
            text=True,
            cwd="."
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
            ["python", "../process_book.py", "embed"],
            capture_output=True,
            text=True,
            cwd="."
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
        docs_path = "../docs"

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