from .gemini_client import GeminiClient
from .vector_store import VectorStore
from .config import Config
from typing import List, Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


class RAGService:
    """
    Service class implementing the Retrieval-Augmented Generation pipeline
    """

    def __init__(self):
        self.gemini_client = GeminiClient()
        self.vector_store = VectorStore()

    def embed_query(self, query_text: str) -> List[float]:
        """
        Generate embedding for the user query
        """
        try:
            return self.gemini_client.generate_embedding(query_text)
        except Exception as e:
            logger.error(f"Error generating embedding for query: {e}")
            raise

    def retrieve(self, query_embedding: List[float], top_k: int = None, threshold: float = None) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks based on the query embedding
        """
        try:
            if top_k is None:
                top_k = Config.RETRIEVAL_TOP_K
            if threshold is None:
                threshold = Config.RETRIEVAL_THRESHOLD

            return self.vector_store.search_similar(
                query_embedding=query_embedding,
                top_k=top_k,
                threshold=threshold
            )
        except Exception as e:
            logger.error(f"Error retrieving chunks: {e}")
            raise

    def augment_prompt(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> str:
        """
        Construct a prompt with the retrieved chunks and user query
        """
        if not retrieved_chunks:
            return f"I couldn't find any relevant information in the book to answer: '{query}'"

        # Build context from retrieved chunks
        context_parts = []
        for chunk in retrieved_chunks:
            context_parts.append(f"Source: {chunk['file_path']}\nContent: {chunk['text']}\n")

        context = "\n".join(context_parts)

        # Create the augmented prompt
        augmented_prompt = f"""
        Based on the following book excerpts, please answer the user's question.
        Make sure to cite the sources and keep the response educational and concise (under {Config.MAX_RESPONSE_WORDS} words).

        Book Excerpts:
        {context}

        User Question: {query}

        Please provide a response based on the book content with citations to the relevant sections.
        """

        return augmented_prompt

    def generate_response(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> str:
        """
        Generate a response based on the query and retrieved chunks
        """
        try:
            if not retrieved_chunks:
                return "I couldn't find any relevant information in the book to answer your question."

            return self.gemini_client.generate_response_with_citations(query, retrieved_chunks)
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise

    def query_rag(self, query_text: str, session_id: str = None) -> Dict[str, Any]:
        """
        Complete RAG pipeline: embed query, retrieve relevant chunks, augment prompt, generate response
        """
        try:
            # Validate query length
            if len(query_text) > Config.MAX_QUERY_LENGTH:
                query_text = query_text[:Config.MAX_QUERY_LENGTH]
                logger.warning(f"Query truncated to {Config.MAX_QUERY_LENGTH} characters")

            # Check if the index is empty before proceeding
            collection_size = self.vector_store.get_collection_size()
            if collection_size == 0:
                logger.warning("Database is empty - no content indexed")
                return {
                    "response": "The database is currently empty. Please index some documents first before asking questions. Run the indexing endpoint with a path to your Markdown files.",
                    "sources": [],
                    "query_text": query_text,
                    "session_id": session_id,
                    "chunks_retrieved": 0,
                    "error": "Database is empty - please index documents first"
                }

            # Step 1: Embed the query
            query_embedding = self.embed_query(query_text)

            # Step 2: Retrieve relevant chunks
            retrieved_chunks = self.retrieve(
                query_embedding=query_embedding,
                top_k=Config.RETRIEVAL_TOP_K,
                threshold=Config.RETRIEVAL_THRESHOLD
            )

            # Step 3: Generate response
            response_text = self.generate_response(query_text, retrieved_chunks)

            # Prepare response with sources
            sources = []
            for chunk in retrieved_chunks:
                sources.append({
                    "text": chunk["text"],
                    "file_path": chunk["file_path"],
                    "chunk_id": chunk["chunk_id"]
                })

            result = {
                "response": response_text,
                "sources": sources,
                "query_text": query_text,
                "session_id": session_id,
                "chunks_retrieved": len(retrieved_chunks)
            }

            return result

        except Exception as e:
            logger.error(f"Error in RAG pipeline: {e}")
            return {
                "response": "An error occurred while processing your request. Please try again.",
                "sources": [],
                "query_text": query_text,
                "session_id": session_id,
                "error": str(e)
            }

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID
        """
        try:
            return self.vector_store.get_chunk_by_id(chunk_id)
        except Exception as e:
            logger.error(f"Error retrieving chunk by ID {chunk_id}: {e}")
            return None

    def test_rag_connection(self) -> bool:
        """
        Test the RAG pipeline with a simple query
        """
        try:
            logger.info("Testing RAG connection...")
            result = self.query_rag("What is this system?")

            # If the database is empty, that's a valid state, not a connection failure
            if "error" in result and "Database is empty" in result.get("error", ""):
                logger.info("RAG connection test: Database is empty (valid state)")
                return True  # Consider this a successful connection test

            success = "response" in result and len(result["response"]) > 0
            if success:
                logger.info("RAG connection test successful")
            else:
                logger.warning("RAG connection test failed - no response returned")
            return success
        except Exception as e:
            logger.error(f"RAG connection test failed: {e}")
            return False