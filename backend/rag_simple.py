"""
Simple RAG pipeline using TF-IDF for the Humanoid Robotics Book
"""
import os
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import logging
from dotenv import load_dotenv
import pickle
import markdown
import glob
import uuid

# Load environment variables
load_dotenv()

# Configuration - only use generation model, embedding will be TF-IDF
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_GENERATION_MODEL = os.getenv("GEMINI_GENERATION_MODEL", "gemini-2.5-flash")  # Updated default
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_vectors")


# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)



def initialize_services():
    """Initialize Qdrant in memory mode and Gemini"""
    # Always use in-memory Qdrant for this version
    qdrant_client = QdrantClient(":memory:")
    logger.info("Qdrant client initialized in memory mode")

    # Initialize Gemini
    if GEMINI_API_KEY:
        genai.configure(api_key=GEMINI_API_KEY)
        logger.info("Gemini API configured successfully")
    else:
        logger.error("GEMINI_API_KEY not found in environment variables")
        return None, None

    return qdrant_client, genai

def embed_query_with_tfidf(query: str) -> List[float]:
    """Generate embedding for a query using the saved TF-IDF vectorizer"""
    try:
        # Validate the query is not empty
        if not query or not query.strip():
            logger.error("Empty query provided for embedding")
            return []

        # Load the saved TF-IDF vectorizer
        try:
            # Try multiple possible locations for the vectorizer file
            possible_paths = [
                "tfidf_vectorizer.pkl",  # Same directory as script (for Docker deployment)
                os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tfidf_vectorizer.pkl"),  # Project root
                os.path.join(os.path.dirname(__file__), "tfidf_vectorizer.pkl"),  # Same directory as script (alternative)
            ]

            vectorizer = None
            vectorizer_path = None

            for path in possible_paths:
                if os.path.exists(path):
                    try:
                        with open(path, 'rb') as f:
                            # Try to load the vectorizer with error handling for compatibility issues
                            import pickle
                            import sys
                            # Set pickle protocol to handle compatibility issues
                            vectorizer = pickle.load(f)
                        vectorizer_path = path
                        break
                    except Exception as load_error:
                        logger.warning(f"Failed to load vectorizer from {path}: {load_error}")
                        continue

            if vectorizer is None:
                logger.error("TF-IDF vectorizer not found or could not be loaded from any expected location.")
                return []

            logger.info(f"Loaded TF-IDF vectorizer from {vectorizer_path}")
        except Exception as e:
            logger.error(f"Error loading TF-IDF vectorizer: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            return []

        # Transform the query using the fitted vectorizer
        query_embedding = vectorizer.transform([query.strip()[:512]])  # Limit to 512 chars
        query_embedding_dense = query_embedding.toarray()[0].tolist()

        logger.info(f"Successfully generated TF-IDF embedding of length {len(query_embedding_dense)}")
        return query_embedding_dense
    except Exception as e:
        logger.error(f"Error generating TF-IDF embedding for query: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return []

def retrieve_from_qdrant(query_embedding: List[float], top_k: int = 3) -> List[Dict[str, Any]]:
    """Retrieve top-k similar documents from Qdrant cloud instance"""
    try:
        # Initialize Qdrant client with cloud settings
        if QDRANT_API_KEY:
            qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        else:
            qdrant_client = QdrantClient(url=QDRANT_URL)

        # Perform search using cosine similarity against cloud collection
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=0.1  # Minimum similarity threshold
        )

        results = []
        for result in search_results:
            results.append({
                "text": result.payload.get("text", ""),
                "source": result.payload.get("source", ""),
                "chunk_id": result.payload.get("chunk_id", ""),
                "score": result.score
            })

        logger.info(f"Retrieved {len(results)} documents from Qdrant cloud with scores: {[r['score'] for r in results]}")
        return results
    except Exception as e:
        logger.error(f"Error retrieving from Qdrant cloud: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
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
    import google.generativeai as genai_local  # Use local import to avoid global state issues

    if not GEMINI_API_KEY:
        return "Error: Gemini API key is not configured. Please set the GEMINI_API_KEY environment variable."

    # Reconfigure the API key to ensure we're using the correct one
    genai_local.configure(api_key=GEMINI_API_KEY)

    max_retries = 3
    for attempt in range(max_retries):
        try:
            # Print the model being used for debugging
            logger.info(f"Using Gemini model: {GEMINI_GENERATION_MODEL}")
            model = genai_local.GenerativeModel(GEMINI_GENERATION_MODEL)
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
        # Validate query input
        if not query or not query.strip():
            return {
                "response": "Please provide a valid question to answer.",
                "sources": [],
                "query": query
            }

        # Truncate very long queries (max 1000 characters as per spec)
        original_query = query
        if len(query) > 1000:
            logger.info(f"Truncating query from {len(query)} to 1000 characters")
            query = query[:1000]

        # Step 1: Embed the query using TF-IDF
        query_embedding = embed_query_with_tfidf(query)
        if not query_embedding:
            logger.error(f"Failed to generate embedding for query: {query[:100]}...")
            return {
                "response": "Error: Could not generate embedding for the query. This might be due to an issue with the AI service or an invalid query. Please try again with a different question.",
                "sources": [],
                "query": original_query
            }

        logger.info(f"Successfully generated embedding of length {len(query_embedding)} for query: {query[:50]}...")

        # Step 2: Retrieve from Qdrant
        retrieved_docs = retrieve_from_qdrant(query_embedding)
        logger.info(f"Retrieved {len(retrieved_docs)} documents from Qdrant")

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
            "query": original_query,
            "retrieved_docs_count": len(retrieved_docs)
        }
    except Exception as e:
        logger.error(f"Error in RAG pipeline: {e}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return {
            "response": f"An error occurred during processing: {str(e)}",
            "sources": [],
            "query": query
        }

def check_services():
    """Check if required services are available"""
    try:
        # Check if TF-IDF vectorizer is available
        tfidf_ok = False
        try:
            # Try multiple possible locations for the vectorizer file
            possible_paths = [
                "tfidf_vectorizer.pkl",  # Same directory as script (for Docker deployment)
                os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tfidf_vectorizer.pkl"),  # Project root
                os.path.join(os.path.dirname(__file__), "tfidf_vectorizer.pkl"),  # Same directory as script (alternative)
            ]

            for path in possible_paths:
                if os.path.exists(path):
                    with open(path, 'rb') as f:
                        import pickle
                        vectorizer = pickle.load(f)
                    tfidf_ok = True
                    break
        except Exception:
            tfidf_ok = False

        return {
            "qdrant": True,  # We always have in-memory
            "tfidf_vectorizer": tfidf_ok,
            "collection": QDRANT_COLLECTION_NAME
        }
    except Exception as e:
        logger.error(f"Error checking services: {e}")
        return {
            "qdrant": False,
            "tfidf_vectorizer": False,
            "error": str(e)
        }

if __name__ == "__main__":
    # Example usage
    sample_query = "What is inverse kinematics in humanoid robots?"
    result = query_rag_pipeline(sample_query)
    print("Response:", result["response"])
    print("Sources:", result["sources"])