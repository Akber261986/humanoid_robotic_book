"""
RAG (Retrieval-Augmented Generation) pipeline for the Humanoid Robotics Book
This module contains functions for embedding queries, retrieving from Qdrant, 
and generating responses with Gemini
"""
import os
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_EMBEDDING_MODEL = os.getenv("GEMINI_EMBEDDING_MODEL", "embedding-001")
GEMINI_GENERATION_MODEL = os.getenv("GEMINI_GENERATION_MODEL", "gemini-1.5-flash")

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def initialize_services():
    """Initialize Qdrant and Gemini services"""
    try:
        # Initialize Qdrant client
        if QDRANT_API_KEY:
            qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        else:
            qdrant_client = QdrantClient(url=QDRANT_URL)

        # Initialize Gemini
        if GEMINI_API_KEY:
            genai.configure(api_key=GEMINI_API_KEY)
        else:
            logger.error("GEMINI_API_KEY not found in environment variables")
            return None, None

        return qdrant_client, genai
    except Exception as e:
        logger.error(f"Error initializing services: {e}")
        return None, None

def embed_query(query: str) -> List[float]:
    """Generate embedding for a query using Gemini"""
    try:
        response = genai.embed_content(
            model=GEMINI_EMBEDDING_MODEL,
            content=[query],
            task_type="retrieval_query"
        )
        return response['embedding'][0]
    except Exception as e:
        logger.error(f"Error generating embedding for query '{query}': {e}")
        return []

def retrieve_from_qdrant(query_embedding: List[float], top_k: int = 3) -> List[Dict[str, Any]]:
    """Retrieve top-k similar documents from Qdrant"""
    try:
        qdrant_client, _ = initialize_services()
        if not qdrant_client:
            return []

        # Search in Qdrant collection
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=0.3  # Minimum similarity threshold
        )

        results = []
        for result in search_results:
            results.append({
                "text": result.payload.get("text", ""),
                "source": result.payload.get("source", ""),
                "chunk_id": result.payload.get("chunk_id", ""),
                "score": result.score
            })

        return results
    except Exception as e:
        logger.error(f"Error retrieving from Qdrant: {e}")
        return []

def augment_prompt(query: str, retrieved_docs: List[Dict[str, Any]]) -> str:
    """Augment the prompt with retrieved documents"""
    if not retrieved_docs:
        return f"""You are an AI assistant for the Humanoid Robotics Book. The user asked: '{query}'

I couldn't find specific information about this topic in the book content. Please provide general information about humanoid robotics that relates to this query. Keep your response educational and relevant to humanoid robotics concepts."""

    # Build context from retrieved documents with source information
    context = ""
    for i, doc in enumerate(retrieved_docs, 1):
        context += f"Document {i} (from {doc['source']}):\n{doc['text']}\n\n"

    prompt = f"""You are an AI assistant for the Humanoid Robotics Book. Answer the user's question based on the following book excerpts.

BOOK EXCERPTS:
{context}

USER QUESTION: {query}

INSTRUCTIONS:
1. Provide a clear, educational response based on the book excerpts
2. If the excerpts contain relevant information, cite the source (e.g., "According to Document 1...")
3. If the information isn't in the excerpts, provide general knowledge about humanoid robotics that relates to the question
4. Keep your response concise but comprehensive
5. Use proper technical terminology where appropriate
6. Structure your response with clear explanations

Please provide your response:"""
    return prompt

def generate_response(prompt: str) -> str:
    """Generate response using Gemini with retry logic"""
    import time

    if not GEMINI_API_KEY:
        return "Error: Gemini API key is not configured. Please set the GEMINI_API_KEY environment variable."

    max_retries = 3
    for attempt in range(max_retries):
        try:
            model = genai.GenerativeModel(GEMINI_GENERATION_MODEL)
            response = model.generate_content(
                prompt,
                generation_config={
                    "temperature": 0.7,
                    "max_output_tokens": 1000,
                    "candidate_count": 1,
                }
            )

            if response.text:
                return response.text
            else:
                return "I couldn't generate a response. Please try rephrasing your question."

        except Exception as e:
            logger.error(f"Attempt {attempt + 1} failed: {e}")
            if attempt == max_retries - 1:  # Last attempt
                error_msg = f"I encountered an error generating a response: {str(e)}. Please try rephrasing your question."
                # If it's an API quota or authentication error, be more specific
                if "quota" in str(e).lower() or "api key" in str(e).lower():
                    error_msg = "API quota exceeded or invalid API key. Please check your Gemini API configuration."
                return error_msg

            # Wait before retrying (exponential backoff)
            time.sleep(2 ** attempt)

def query_rag_pipeline(query: str) -> Dict[str, Any]:
    """Complete RAG pipeline: embed query, retrieve, augment, and generate"""
    try:
        # Truncate very long queries (max 1000 characters as per spec)
        if len(query) > 1000:
            logger.info(f"Truncating query from {len(query)} to 1000 characters")
            query = query[:1000]

        # Step 1: Embed the query
        query_embedding = embed_query(query)
        if not query_embedding:
            return {
                "response": "Error: Could not generate embedding for the query. Please try again with a different question.",
                "sources": [],
                "query": query
            }

        # Step 2: Retrieve from Qdrant
        retrieved_docs = retrieve_from_qdrant(query_embedding)

        # Step 3: Augment prompt with retrieved documents
        augmented_prompt = augment_prompt(query, retrieved_docs)

        # Step 4: Generate response
        response = generate_response(augmented_prompt)

        # Prepare sources for response
        sources = []
        for doc in retrieved_docs:
            source_text = doc["text"]
            # Limit text length and ensure it's not empty
            if source_text and len(source_text) > 0:
                preview = source_text[:200] + "..." if len(source_text) > 200 else source_text
                sources.append({
                    "text": preview,
                    "file": doc["source"],
                    "score": round(doc["score"], 3)  # Round similarity score for readability
                })

        return {
            "response": response,
            "sources": sources,
            "query": query,
            "retrieved_docs_count": len(retrieved_docs)
        }
    except Exception as e:
        logger.error(f"Error in RAG pipeline: {e}")
        return {
            "response": f"An error occurred during processing: {str(e)}",
            "sources": [],
            "query": query
        }

# Health check function
def check_services():
    """Check if required services are available"""
    try:
        qdrant_client, genai_client = initialize_services()
        
        # Test Qdrant connection
        qdrant_ok = False
        if qdrant_client:
            try:
                qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
                qdrant_ok = True
            except:
                qdrant_ok = False
        
        # Test Gemini (just check if API key is set)
        gemini_ok = bool(GEMINI_API_KEY)
        
        return {
            "qdrant": qdrant_ok,
            "gemini_api": gemini_ok,
            "collection": QDRANT_COLLECTION_NAME if qdrant_ok else None
        }
    except Exception as e:
        logger.error(f"Error checking services: {e}")
        return {
            "qdrant": False,
            "gemini_api": False,
            "error": str(e)
        }

if __name__ == "__main__":
    # Example usage
    sample_query = "What is inverse kinematics in humanoid robots?"
    result = query_rag_pipeline(sample_query)
    print("Response:", result["response"])
    print("Sources:", result["sources"])