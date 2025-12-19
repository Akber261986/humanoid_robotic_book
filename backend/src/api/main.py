from fastapi import FastAPI, HTTPException, Request, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import logging
import uuid
import asyncio
from datetime import datetime
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from .services.rag_service import RAGService
from .services.index_service import IndexService
from .services.config import Config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize services
rag_service = RAGService()
index_service = IndexService()

app = FastAPI(title="Humanoid Robotics RAG Chatbot API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    query: str
    session_id: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]
    query_id: str

class IndexResponse(BaseModel):
    status: str
    chunks_indexed: int
    files_processed: int

@app.get("/")
async def root():
    return {
        "message": "Humanoid Robotics RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": [
            "/query - Process user queries",
            "/index - Index book content",
            "/upload - Upload and index files",
            "/health - Health check"
        ]
    }

@app.get("/health")
async def health_check():
    try:
        # Test RAG service connection
        rag_connected = rag_service.test_connection()
        # Test index service connection
        index_connected = index_service.test_connection()

        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat(),
            "services": {
                "rag": "connected" if rag_connected else "disconnected",
                "index": "connected" if index_connected else "disconnected",
                "qdrant": "connected" if rag_service.qdrant_connected() else "disconnected",
                "gemini": "connected" if rag_service.gemini_connected() else "disconnected"
            }
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {
            "status": "unhealthy",
            "error": str(e),
            "timestamp": datetime.now().isoformat()
        }

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        result = rag_service.query(request.query, request.session_id)
        return QueryResponse(
            response=result["response"],
            sources=result["sources"],
            query_id=str(uuid.uuid4())
        )
    except Exception as e:
        logger.error(f"Query error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/index")
async def index_endpoint():
    try:
        result = index_service.index_docs()
        return IndexResponse(
            status=result["status"],
            chunks_indexed=result["chunks_indexed"],
            files_processed=result["files_processed"]
        )
    except Exception as e:
        logger.error(f"Index error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/upload")
async def upload_endpoint(file: UploadFile = File(...)):
    try:
        # Save uploaded file temporarily
        file_path = f"temp_{file.filename}"
        with open(file_path, "wb") as buffer:
            buffer.write(await file.read())

        # Index the uploaded file
        result = index_service.index_single_file(file_path)

        # Clean up temporary file
        os.remove(file_path)

        return IndexResponse(
            status=result["status"],
            chunks_indexed=result["chunks_indexed"],
            files_processed=1
        )
    except Exception as e:
        logger.error(f"Upload error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)