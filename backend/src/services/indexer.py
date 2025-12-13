from ..utils.markdown_parser import MarkdownParser
from ..utils.chunking import create_chunks_from_text
from .vector_store import VectorStore
from .gemini_client import GeminiClient
from .config import Config
from typing import List
import os
import logging

logger = logging.getLogger(__name__)


class Indexer:
    """
    Service class to handle content indexing from Markdown files into the vector database
    """

    def __init__(self):
        self.vector_store = VectorStore()
        self.gemini_client = GeminiClient()
        self.parser = MarkdownParser()

    def index_directory(self, docs_path: str) -> dict:
        """
        Index all Markdown files in the given directory
        """
        logger.info(f"Starting indexing process for directory: {docs_path}")

        # Get all markdown files
        markdown_files = self.parser.get_all_markdown_files(docs_path)
        logger.info(f"Found {len(markdown_files)} markdown files to process")

        total_chunks = 0
        processed_files = 0

        for file_path in markdown_files:
            try:
                logger.info(f"Processing file: {file_path}")

                # Extract text from markdown file
                text_content = self.parser.extract_text_from_md(file_path)

                # Create chunks from the text
                chunks = create_chunks_from_text(
                    text_content,
                    file_path,
                    Config.CHUNK_SIZE_MIN,
                    Config.CHUNK_SIZE_MAX
                )

                # Generate embeddings for all chunks
                texts_to_embed = [chunk.text for chunk in chunks]
                embeddings = self.gemini_client.generate_embeddings_batch(texts_to_embed)

                # Assign embeddings to chunks
                for i, chunk in enumerate(chunks):
                    chunk.embedding = embeddings[i]

                # Store chunks in vector database
                self.vector_store.store_chunks(chunks)

                total_chunks += len(chunks)
                processed_files += 1

                logger.info(f"Processed {len(chunks)} chunks from {file_path}")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
                # Continue with other files
                continue

        result = {
            "status": "completed",
            "chunks_indexed": total_chunks,
            "files_processed": processed_files,
            "total_files_found": len(markdown_files)
        }

        logger.info(f"Indexing completed: {result}")
        return result

    def index_single_file(self, file_path: str) -> dict:
        """
        Index a single Markdown file
        """
        logger.info(f"Processing single file: {file_path}")

        # Extract text from markdown file
        text_content = self.parser.extract_text_from_md(file_path)

        # Create chunks from the text
        chunks = create_chunks_from_text(
            text_content,
            file_path,
            Config.CHUNK_SIZE_MIN,
            Config.CHUNK_SIZE_MAX
        )

        # Generate embeddings for all chunks
        texts_to_embed = [chunk.text for chunk in chunks]
        embeddings = self.gemini_client.generate_embeddings_batch(texts_to_embed)

        # Assign embeddings to chunks
        for i, chunk in enumerate(chunks):
            chunk.embedding = embeddings[i]

        # Store chunks in vector database
        self.vector_store.store_chunks(chunks)

        result = {
            "status": "completed",
            "chunks_indexed": len(chunks),
            "file_processed": file_path
        }

        logger.info(f"Single file indexing completed: {result}")
        return result

    def check_index_status(self) -> dict:
        """
        Check the status of the index
        """
        collection_size = self.vector_store.get_collection_size()

        status = {
            "collection_name": Config.QDRANT_COLLECTION_NAME,
            "total_chunks": collection_size,
            "indexed": collection_size > 0,
            "qdrant_url": Config.QDRANT_URL,
            "needs_indexing": collection_size == 0,
            "message": "Database is empty - please index documents first" if collection_size == 0 else "Database contains indexed content"
        }

        return status

    def clear_index(self):
        """
        Clear the entire index (useful for re-indexing)
        """
        logger.info("Clearing the entire index")
        self.vector_store.clear_collection()
        logger.info("Index cleared successfully")