import logging
import os
from typing import List, Dict, Any
import markdown
from qdrant_client.http import models
from .rag_service import RAGService
from .config import Config

logger = logging.getLogger(__name__)

class IndexService:
    def __init__(self):
        self.rag_service = RAGService()

    def test_connection(self):
        """Test if the service can connect properly"""
        return self.rag_service.test_connection()

    def chunk_text(self, text: str, chunk_size: int = Config.CHUNK_SIZE, overlap: int = Config.OVERLAP_SIZE) -> List[str]:
        """Split text into overlapping chunks"""
        sentences = text.split('. ')
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            if len(current_chunk + sentence) < chunk_size:
                current_chunk += sentence + ". "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())

                # Create overlapping chunk
                words = current_chunk.split()
                overlap_start = max(0, len(words) - overlap)
                current_chunk = " ".join(words[overlap_start:]) + " " + sentence + ". "

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def extract_text_from_md(self, file_path: str) -> str:
        """Extract text content from a markdown file"""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # Convert markdown to plain text (remove markdown formatting)
            html = markdown.markdown(content)
            # Simple approach to extract text from HTML
            text = html.replace('<p>', ' ').replace('</p>', ' ').replace('<h1>', ' ').replace('</h1>', ' ') \
                      .replace('<h2>', ' ').replace('</h2>', ' ').replace('<h3>', ' ').replace('</h3>', ' ') \
                      .replace('<li>', ' ').replace('</li>', ' ').replace('<ul>', ' ').replace('</ul>', ' ') \
                      .replace('<strong>', ' ').replace('</strong>', ' ').replace('<em>', ' ').replace('</em>', ' ')

            # Remove extra whitespace
            import re
            text = re.sub(r'\s+', ' ', text).strip()

            return text
        except Exception as e:
            logger.error(f"Error reading markdown file {file_path}: {e}")
            return ""

    def index_docs(self) -> Dict[str, Any]:
        """Index all markdown files in the docs directory"""
        docs_path = Config.DOCS_PATH

        if not os.path.exists(docs_path):
            raise FileNotFoundError(f"Docs path does not exist: {docs_path}")

        markdown_files = []
        for root, dirs, files in os.walk(docs_path):
            for file in files:
                if file.lower().endswith('.md'):
                    markdown_files.append(os.path.join(root, file))

        logger.info(f"Found {len(markdown_files)} markdown files to index")

        total_chunks = 0
        processed_files = 0

        for file_path in markdown_files:
            try:
                logger.info(f"Processing file: {file_path}")

                # Extract text from markdown
                text_content = self.extract_text_from_md(file_path)

                if not text_content.strip():
                    logger.warning(f"No content extracted from {file_path}")
                    continue

                # Split into chunks
                chunks = self.chunk_text(text_content)

                # Process each chunk
                points = []
                for i, chunk in enumerate(chunks):
                    if not chunk.strip():
                        continue

                    # Generate embedding
                    embedding = self.rag_service.embed_text(chunk)

                    # Create Qdrant point
                    point = models.PointStruct(
                        id=f"{os.path.basename(file_path)}_{i}",
                        vector=embedding,
                        payload={
                            "text": chunk,
                            "source": file_path,
                            "chunk_id": i
                        }
                    )
                    points.append(point)

                # Upload to Qdrant
                if points:
                    self.rag_service.qdrant_client.upsert(
                        collection_name=self.rag_service.collection_name,
                        points=points
                    )

                    total_chunks += len(points)
                    logger.info(f"Indexed {len(points)} chunks from {file_path}")

                processed_files += 1

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
                continue

        return {
            "status": "completed",
            "chunks_indexed": total_chunks,
            "files_processed": processed_files,
            "total_files_found": len(markdown_files)
        }

    def index_single_file(self, file_path: str) -> Dict[str, Any]:
        """Index a single markdown file"""
        try:
            logger.info(f"Processing single file: {file_path}")

            # Extract text from markdown
            text_content = self.extract_text_from_md(file_path)

            if not text_content.strip():
                return {
                    "status": "no content",
                    "chunks_indexed": 0,
                    "files_processed": 0
                }

            # Split into chunks
            chunks = self.chunk_text(text_content)

            # Process each chunk
            points = []
            for i, chunk in enumerate(chunks):
                if not chunk.strip():
                    continue

                # Generate embedding
                embedding = self.rag_service.embed_text(chunk)

                # Create Qdrant point
                point = models.PointStruct(
                    id=f"{os.path.basename(file_path)}_{i}",
                    vector=embedding,
                    payload={
                        "text": chunk,
                        "source": file_path,
                        "chunk_id": i
                    }
                )
                points.append(point)

            # Upload to Qdrant
            if points:
                self.rag_service.qdrant_client.upsert(
                    collection_name=self.rag_service.collection_name,
                    points=points
                )

            return {
                "status": "completed",
                "chunks_indexed": len(points),
                "files_processed": 1
            }

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            raise