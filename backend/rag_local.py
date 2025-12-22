"""
RAG (Retrieval-Augmented Generation) pipeline for the Humanoid Robotics Book
This module contains functions for embedding queries, retrieving from Qdrant,
and generating responses with Gemini - with local fallback
"""
import os
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import logging
from dotenv import load_dotenv
import pickle

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_vectors")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_EMBEDDING_MODEL = os.getenv("GEMINI_EMBEDDING_MODEL", "models/embedding-001")
GEMINI_GENERATION_MODEL = os.getenv("GEMINI_GENERATION_MODEL", "gemini-1.5-pro-latest")

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def initialize_services():
    """Initialize Qdrant and Gemini services with local fallback"""
    try:
        # Try to initialize with remote server first
        if QDRANT_API_KEY:
            qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
            logger.info(f"Qdrant client initialized with API key, connecting to {QDRANT_URL}")
        else:
            qdrant_client = QdrantClient(url=QDRANT_URL)
            logger.info(f"Qdrant client initialized without API key, connecting to {QDRANT_URL}")

        # Test the connection
        try:
            qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            logger.info("Successfully connected to remote Qdrant server")
        except Exception as e:
            logger.warning(f"Could not connect to remote Qdrant: {e}. Using in-memory fallback.")
            qdrant_client = QdrantClient(":memory:")
            logger.info("Qdrant client initialized in memory mode")

    except Exception as e:
        logger.warning(f"Error connecting to remote Qdrant: {e}. Using in-memory fallback.")
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
            # Try multiple possible paths for the vectorizer
            vectorizer_paths = [
                "../../tfidf_vectorizer.pkl",  # Relative to backend from rag_local
                "../tfidf_vectorizer.pkl",     # Alternative path
                "tfidf_vectorizer.pkl",        # Same directory as rag_local
                "../tfidf_vectorizer.pkl",     # From backend directory
                "../../tfidf_vectorizer.pkl"   # Original path
            ]

            vectorizer = None
            for path in vectorizer_paths:
                try:
                    full_path = os.path.join(os.path.dirname(__file__), path)
                    with open(full_path, 'rb') as f:
                        vectorizer = pickle.load(f)
                    logger.info(f"Loaded TF-IDF vectorizer from {full_path}")
                    break
                except FileNotFoundError:
                    continue

            if vectorizer is None:
                # Try absolute path from project root
                project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                vectorizer_path = os.path.join(project_root, "tfidf_vectorizer.pkl")
                with open(vectorizer_path, 'rb') as f:
                    vectorizer = pickle.load(f)
                logger.info(f"Loaded TF-IDF vectorizer from {vectorizer_path}")

        except FileNotFoundError:
            logger.error("TF-IDF vectorizer not found in any expected location. Run embedding script first.")
            logger.error(f"Expected path: {os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tfidf_vectorizer.pkl')}")
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
            score_threshold=0.01  # Lower threshold for TF-IDF
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

# Health check function
def check_services():
    """Check if required services are available"""
    try:
        qdrant_client, _ = initialize_services()

        # Test Qdrant connection
        qdrant_ok = False
        if qdrant_client:
            try:
                qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
                qdrant_ok = True
            except:
                # For in-memory, we'll just check if we can create/access it
                try:
                    # Try to create the collection if it doesn't exist
                    qdrant_client.create_collection(
                        collection_name=QDRANT_COLLECTION_NAME,
                        vectors_config=models.VectorParams(size=1000, distance=models.Distance.COSINE),
                    )
                    qdrant_ok = True
                except:
                    # If we can't create it, we can still try to work with it
                    qdrant_ok = True

        # Check if TF-IDF vectorizer is available
        try:
            # Try multiple possible paths for the vectorizer
            vectorizer_paths = [
                "../../tfidf_vectorizer.pkl",  # Relative to backend from rag_local
                "../tfidf_vectorizer.pkl",     # Alternative path
                "tfidf_vectorizer.pkl",        # Same directory as rag_local
                "../tfidf_vectorizer.pkl",     # From backend directory
                "../../tfidf_vectorizer.pkl"   # Original path
            ]

            vectorizer = None
            for path in vectorizer_paths:
                try:
                    full_path = os.path.join(os.path.dirname(__file__), path)
                    with open(full_path, 'rb') as f:
                        vectorizer = pickle.load(f)
                    logger.info(f"Found TF-IDF vectorizer at {full_path}")
                    break
                except FileNotFoundError:
                    continue

            if vectorizer is None:
                # Try absolute path from project root
                project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                vectorizer_path = os.path.join(project_root, "tfidf_vectorizer.pkl")
                with open(vectorizer_path, 'rb') as f:
                    vectorizer = pickle.load(f)
                logger.info(f"Found TF-IDF vectorizer at {vectorizer_path}")

            tfidf_ok = True
        except FileNotFoundError:
            tfidf_ok = False

        return {
            "qdrant": qdrant_ok,
            "tfidf_vectorizer": tfidf_ok,
            "collection": QDRANT_COLLECTION_NAME if qdrant_ok else None
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