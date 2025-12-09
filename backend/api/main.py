#!/usr/bin/env python3
"""
FastAPI backend with Google Generative AI integration for the Physical AI & Humanoid Robotics book chatbot.
This backend provides endpoints for RAG-based question answering using Qdrant and Google Gemini.
"""
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import os
import logging
from qdrant_client import QdrantClient
import google.generativeai as genai
from dotenv import load_dotenv
import asyncio
import json

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure Google Gemini API
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://7118c77f-72dd-44cc-ae76-7845348d45e0.europe-west3-0.gcp.cloud.qdrant.io")  # Update with your Qdrant cluster URL
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_book")

if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY environment variable is required")

genai.configure(api_key=GEMINI_API_KEY)

# Initialize clients
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    prefer_grpc=True
)

app = FastAPI(
    title="Physical AI & Humanoid Robotics Book RAG API",
    description="API for question answering based on the Physical AI & Humanoid Robotics book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    selected_text: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict[str, Any]]
    query: str

class HealthResponse(BaseModel):
    status: str
    collections: List[str]

def get_embedding(text: str) -> List[float]:
    """Get embedding for text using Google Gemini embedding-001 model."""
    try:
        result = genai.embed_content(
            model="models/embedding-001",
            content=text,
            task_type="retrieval_query"  # Using retrieval_query for search queries
        )
        return result['embedding']
    except Exception as e:
        logger.error(f"Error getting embedding for text: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error getting embedding: {str(e)}")

def retrieve_context(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """Retrieve relevant context from Qdrant based on the query."""
    try:
        # Get embedding for the query
        query_embedding = get_embedding(query)

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Extract context and metadata
        contexts = []
        for result in search_results:
            contexts.append({
                "content": result.payload["content"],
                "source": result.payload["source"],
                "file_path": result.payload["file_path"],
                "chunk_index": result.payload["chunk_index"],
                "score": result.score
            })

        return contexts
    except Exception as e:
        logger.error(f"Error retrieving context: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error retrieving context: {str(e)}")

def generate_answer(query: str, context: List[Dict[str, Any]], selected_text: Optional[str] = None) -> str:
    """Generate an answer using Google Gemini based on the query and context."""
    try:
        # Prepare the context for the model
        context_str = "\n\n".join([f"Source: {ctx['source']}\nContent: {ctx['content'][:500]}..." for ctx in context])

        # Prepare the prompt based on whether there's selected text
        if selected_text:
            prompt = f"""
            The user has selected the following text from the book:
            "{selected_text}"

            The user wants an explanation about this text.

            Book context:
            {context_str}

            Please provide a detailed explanation of the selected text based on the book content.
            """
        else:
            prompt = f"""
            Question: {query}

            Context from the Physical AI & Humanoid Robotics book:
            {context_str}

            Please answer the question based on the provided context from the book.
            If the context doesn't contain the information needed to answer the question,
            please state that the information is not available in the current context.
            """

        # Use the Gemini model to generate the answer
        model = genai.GenerativeModel('gemini-2.5-flash')
        response = model.generate_content(prompt)

        return response.text
    except Exception as e:
        logger.error(f"Error generating answer: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error generating answer: {str(e)}")

@app.get("/")
def read_root():
    """Health check endpoint."""
    return {"message": "Physical AI & Humanoid Robotics Book RAG API is running"}

@app.get("/health", response_model=HealthResponse)
def health_check():
    """Health check with Qdrant status."""
    try:
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]
        return HealthResponse(status="healthy", collections=collection_names)
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")

@app.post("/query", response_model=QueryResponse)
def query_endpoint(request: QueryRequest):
    """Main endpoint for querying the book content."""
    try:
        # Retrieve relevant context from Qdrant
        contexts = retrieve_context(request.query, request.top_k)

        # Generate answer based on context
        answer = generate_answer(request.query, contexts, request.selected_text)

        return QueryResponse(
            answer=answer,
            sources=contexts,
            query=request.query
        )
    except Exception as e:
        logger.error(f"Query endpoint error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/explain-selected")
def explain_selected_endpoint(request: QueryRequest):
    """Endpoint for explaining selected text from the book."""
    if not request.selected_text:
        raise HTTPException(status_code=400, detail="selected_text is required for this endpoint")

    try:
        # Retrieve relevant context from Qdrant
        contexts = retrieve_context(request.selected_text, request.top_k)

        # Generate explanation based on the selected text and context
        answer = generate_answer(request.query, contexts, request.selected_text)

        return QueryResponse(
            answer=answer,
            sources=contexts,
            query=request.selected_text
        )
    except Exception as e:
        logger.error(f"Explain selected endpoint error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error explaining selected text: {str(e)}")

@app.get("/collections")
def list_collections():
    """List available Qdrant collections."""
    try:
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]
        return {"collections": collection_names}
    except Exception as e:
        logger.error(f"Error listing collections: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error listing collections: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    import os
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)