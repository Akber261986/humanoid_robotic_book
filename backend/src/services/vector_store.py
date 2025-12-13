from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
from ..models.chunk import BookContentChunk
from .config import Config
import logging

logger = logging.getLogger(__name__)


class VectorStore:
    """
    Service class to handle vector storage operations using Qdrant
    """

    def __init__(self):
        self._client = None
        self.collection_name = Config.QDRANT_COLLECTION_NAME
        self.url = Config.QDRANT_URL

    @property
    def client(self):
        if self._client is None:
            self._client = QdrantClient(url=self.url)
            # Only try to ensure collection exists when client is first used
            self._ensure_collection_exists()
        return self._client

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists with the proper configuration
        """
        try:
            # Check if collection exists
            self.client.get_collection(collection_name=self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.warning(f"Could not connect to Qdrant or collection doesn't exist: {e}")
            # We can't create the collection if Qdrant is not available
            # This is expected when Qdrant is not running
            pass

    def store_chunks(self, chunks: List[BookContentChunk]):
        """
        Store multiple content chunks in the vector database
        """
        points = []
        for chunk in chunks:
            if chunk.embedding is None:
                logger.warning(f"Chunk {chunk.chunk_id} has no embedding, skipping")
                continue

            points.append(
                models.PointStruct(
                    id=chunk.chunk_id,
                    vector=chunk.embedding,
                    payload={
                        "text": chunk.text,
                        "file_path": chunk.file_path,
                        "heading": chunk.heading,
                        "metadata": chunk.metadata or {}
                    }
                )
            )

        if points:
            try:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                logger.info(f"Stored {len(points)} chunks in vector database")
            except Exception as e:
                logger.error(f"Error storing chunks: {e}")
                raise

    def search_similar(self, query_embedding: List[float], top_k: int = 3, threshold: float = 0.7) -> List[Dict[str, Any]]:
        """
        Search for similar content chunks based on the query embedding
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold
            )

            similar_chunks = []
            for result in results:
                similar_chunks.append({
                    "chunk_id": result.id,
                    "text": result.payload.get("text", ""),
                    "file_path": result.payload.get("file_path", ""),
                    "heading": result.payload.get("heading"),
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {})
                })

            return similar_chunks
        except Exception as e:
            logger.error(f"Error searching for similar chunks: {e}")
            return []

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID
        """
        try:
            results = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )

            if results:
                result = results[0]
                return {
                    "chunk_id": result.id,
                    "text": result.payload.get("text", ""),
                    "file_path": result.payload.get("file_path", ""),
                    "heading": result.payload.get("heading"),
                    "metadata": result.payload.get("metadata", {})
                }

            return None
        except Exception as e:
            logger.error(f"Error retrieving chunk by ID {chunk_id}: {e}")
            return None

    def clear_collection(self):
        """
        Clear all vectors from the collection (useful for re-indexing)
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            self._ensure_collection_exists()
            logger.info(f"Cleared and recreated collection {self.collection_name}")
        except Exception as e:
            logger.error(f"Error clearing collection: {e}")
            raise

    def get_collection_size(self) -> int:
        """
        Get the number of vectors in the collection
        """
        try:
            collection_info = self.client.get_collection(collection_name=self.collection_name)
            return collection_info.points_count
        except Exception as e:
            logger.warning(f"Could not get collection size: {e}")
            return 0