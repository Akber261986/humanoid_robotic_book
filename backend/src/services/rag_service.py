import logging
from typing import List, Dict, Any, Optional
from google.generativeai import embedding as genai_embedding
from google.generativeai import GenerativeModel
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from .config import Config

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        # Initialize Gemini
        genai.configure(api_key=Config.GEMINI_API_KEY)
        self.embedding_model = Config.GEMINI_EMBEDDING_MODEL
        self.generative_model = GenerativeModel(Config.GEMINI_GENERATION_MODEL)

        # Initialize Qdrant
        if Config.QDRANT_API_KEY:
            self.qdrant_client = QdrantClient(
                url=Config.QDRANT_URL,
                api_key=Config.QDRANT_API_KEY,
            )
        else:
            self.qdrant_client = QdrantClient(url=Config.QDRANT_URL)

        self.collection_name = Config.QDRANT_COLLECTION_NAME
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the Qdrant collection exists"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
            )
            logger.info(f"Created collection {self.collection_name}")

    def test_connection(self):
        """Test if the service can connect properly"""
        try:
            # Test Qdrant connection
            self.qdrant_client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"RAG service connection test failed: {e}")
            return False

    def qdrant_connected(self):
        """Check if Qdrant is connected"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
            return True
        except:
            return False

    def gemini_connected(self):
        """Check if Gemini is connected"""
        try:
            # Try to get embeddings for a test text
            test_text = "test"
            genai_embedding.embed_content(
                model=self.embedding_model,
                content=[test_text],
                task_type="retrieval_document"
            )
            return True
        except:
            return False

    def embed_text(self, text: str) -> List[float]:
        """Generate embedding for text using Gemini"""
        try:
            response = genai_embedding.embed_content(
                model=self.embedding_model,
                content=[text],
                task_type="retrieval_document"
            )
            return response['embedding'][0]
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def query(self, query_text: str, session_id: Optional[str] = None) -> Dict[str, Any]:
        """Process a query using RAG"""
        try:
            # Generate embedding for the query
            query_embedding = self.embed_text(query_text)

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=Config.TOP_K,
                score_threshold=Config.SIMILARITY_THRESHOLD
            )

            # Prepare context from retrieved documents
            context_parts = []
            sources = []
            for result in search_results:
                if result.score >= Config.SIMILARITY_THRESHOLD:
                    context_parts.append(result.payload.get('text', ''))
                    sources.append({
                        'text': result.payload.get('text', '')[:200] + "...",
                        'source': result.payload.get('source', 'Unknown'),
                        'score': result.score
                    })

            context = "\n\n".join(context_parts)

            # Generate response using Gemini
            if context:
                prompt = f"""
                Based on the following context from the Humanoid Robotics book, answer the question.
                If the context doesn't contain relevant information, say so.

                Context: {context}

                Question: {query_text}

                Answer:
                """
            else:
                prompt = f"Please answer the following question about humanoid robotics: {query_text}"

            response = self.generative_model.generate_content(prompt)
            answer = response.text if response.text else "I couldn't find relevant information to answer your question."

            return {
                "response": answer,
                "sources": sources,
                "query": query_text,
                "session_id": session_id
            }

        except Exception as e:
            logger.error(f"Error in query: {e}")
            return {
                "response": "An error occurred while processing your query. Please try again.",
                "sources": [],
                "query": query_text,
                "session_id": session_id,
                "error": str(e)
            }