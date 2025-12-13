from fastapi import FastAPI, HTTPException, Request, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
import logging
import uuid
import time
import asyncio
from collections import defaultdict

from ..services.rag import RAGService
from ..services.indexer import Indexer
from ..services.config import Config
from ..services.session_manager import session_manager, ChatSession

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize services
rag_service = RAGService()
indexer = Indexer()

# Rate limiting storage (in a real app, you'd use Redis or similar)
request_counts = defaultdict(list)

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class QueryRequest(BaseModel):
    query: str = Field(..., max_length=Config.MAX_QUERY_LENGTH, description="The user's question about humanoid robotics")
    session_id: str = Field(..., description="The chat session identifier")


class QueryResponse(BaseModel):
    response: str = Field(..., description="The AI-generated answer to the query")
    sources: List[Dict[str, Any]] = Field(..., description="Array of source chunks used to generate the response")
    query_id: str = Field(..., description="Unique identifier for the query")
    response_id: str = Field(..., description="Unique identifier for the response")


class IndexRequest(BaseModel):
    docs_path: str = Field(..., description="Path to the directory containing Markdown files")


class IndexResponse(BaseModel):
    status: str = Field(..., description="Status of the indexing operation")
    chunks_indexed: int = Field(..., description="Number of chunks that were indexed")
    files_processed: int = Field(..., description="Number of files that were processed")


class CreateSessionResponse(BaseModel):
    session_id: str = Field(..., description="Unique identifier for the created session")
    created_at: str = Field(..., description="Timestamp when the session was created")


class SessionDetailsResponse(BaseModel):
    session_id: str = Field(..., description="Session identifier")
    created_at: str = Field(..., description="When the session was created")
    last_accessed: str = Field(..., description="When the session was last accessed")
    query_count: int = Field(..., description="Number of queries in this session")


class HealthResponse(BaseModel):
    status: str = Field(..., description="Health status of the service")
    timestamp: str = Field(..., description="Current timestamp")
    dependencies: Dict[str, str] = Field(..., description="Status of dependencies")


def check_rate_limit(request: Request) -> bool:
    """
    Check if the request exceeds rate limits
    """
    client_ip = request.client.host if request.client else "unknown"
    current_time = time.time()

    # Clean old requests (older than the window)
    request_counts[client_ip] = [
        req_time for req_time in request_counts[client_ip]
        if current_time - req_time < Config.RATE_LIMIT_WINDOW
    ]

    # Check if limit exceeded
    if len(request_counts[client_ip]) >= Config.RATE_LIMIT_REQUESTS:
        return False

    # Add current request
    request_counts[client_ip].append(current_time)
    return True


@app.post("/api/query", response_model=QueryResponse)
async def query_endpoint(query_request: QueryRequest, request: Request):
    """
    Process a user query and return a RAG-enhanced response with citations.
    """
    try:
        # Check rate limit
        if not check_rate_limit(request):
            raise HTTPException(status_code=429, detail="Rate limit exceeded")

        # Validate inputs
        if not query_request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        # Process the query through RAG pipeline
        result = rag_service.query_rag(query_request.query, query_request.session_id)

        if "error" in result:
            raise HTTPException(status_code=500, detail=result["error"])

        # Generate IDs
        query_id = str(uuid.uuid4())
        response_id = str(uuid.uuid4())

        # Add query to session if session exists
        if query_request.session_id:
            session_manager.add_query_to_session(query_request.session_id, query_id)

        return QueryResponse(
            response=result["response"],
            sources=result["sources"],
            query_id=query_id,
            response_id=response_id
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/api/index", response_model=IndexResponse)
async def index_endpoint(request: IndexRequest):
    """
    Index the book content from Markdown files into the vector database.
    """
    try:
        # Validate inputs
        import os
        if not os.path.exists(request.docs_path):
            raise HTTPException(status_code=400, detail="docs_path does not exist")

        # Perform indexing
        result = indexer.index_directory(request.docs_path)

        return IndexResponse(
            status=result["status"],
            chunks_indexed=result["chunks_indexed"],
            files_processed=result["files_processed"]
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error during indexing: {e}")
        raise HTTPException(status_code=500, detail="Indexing failed")


@app.post("/api/session", response_model=CreateSessionResponse)
async def create_session(request: Request):
    """
    Create a new chat session.
    """
    try:
        # Check rate limit
        if not check_rate_limit(request):
            raise HTTPException(status_code=429, detail="Rate limit exceeded")

        session = session_manager.create_session()
        return CreateSessionResponse(
            session_id=session.session_id,
            created_at=session.created_at.isoformat()
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error creating session: {e}")
        raise HTTPException(status_code=500, detail="Failed to create session")


@app.get("/api/session/{session_id}", response_model=SessionDetailsResponse)
async def get_session(session_id: str, request: Request):
    """
    Get session details.
    """
    try:
        # Check rate limit
        if not check_rate_limit(request):
            raise HTTPException(status_code=429, detail="Rate limit exceeded")

        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        return SessionDetailsResponse(
            session_id=session.session_id,
            created_at=session.created_at.isoformat(),
            last_accessed=session.last_accessed.isoformat(),
            query_count=len(session.queries)
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error getting session details: {e}")
        raise HTTPException(status_code=500, detail="Failed to get session details")


@app.get("/api/chunk/{chunk_id}")
async def get_chunk_by_id(chunk_id: str, request: Request):
    """
    Retrieve a specific chunk by its ID for citation expansion.
    """
    try:
        # Check rate limit
        if not check_rate_limit(request):
            raise HTTPException(status_code=429, detail="Rate limit exceeded")

        chunk_data = rag_service.get_chunk_by_id(chunk_id)
        if chunk_data is None:
            raise HTTPException(status_code=404, detail="Chunk not found")
        return chunk_data
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error getting chunk by ID: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve chunk")


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the backend service.
    """
    try:
        # Test dependencies
        qdrant_status = "connected" if indexer.check_index_status() else "disconnected"
        gemini_status = "available" if rag_service.test_rag_connection() else "unavailable"

        return HealthResponse(
            status="healthy",
            timestamp=datetime.now().isoformat(),
            dependencies={
                "qdrant": qdrant_status,
                "gemini_api": gemini_status
            }
        )
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthResponse(
            status="unhealthy",
            timestamp=datetime.now().isoformat(),
            dependencies={
                "qdrant": "error",
                "gemini_api": "error"
            }
        )


@app.get("/")
async def root():
    """
    Root endpoint for basic service information.
    """
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": [
            "/api/query - Process user queries",
            "/api/index - Index book content",
            "/api/session - Manage chat sessions",
            "/health - Health check"
        ]
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)