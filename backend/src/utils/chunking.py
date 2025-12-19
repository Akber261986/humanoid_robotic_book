import re
from typing import List, Tuple
from ..models.chunk import BookContentChunk
from ..services.config import Config
import uuid


class TextChunker:
    """
    Utility class for splitting text into semantic chunks of specified size
    """

    @staticmethod
    def chunk_text(text: str, min_size: int = None, max_size: int = None) -> List[Tuple[str, str]]:
        """
        Split text into chunks between min_size and max_size characters
        Returns list of (chunk_id, chunk_text) tuples
        """
        if min_size is None:
            min_size = Config.CHUNK_SIZE_MIN
        if max_size is None:
            max_size = Config.CHUNK_SIZE_MAX

        chunks = []
        paragraphs = re.split(r'\n\s*\n', text)  # Split by paragraph breaks

        current_chunk = ""
        current_heading = ""

        for paragraph in paragraphs:
            # If paragraph is a heading (starts with # or is short and likely a heading)
            if paragraph.strip().startswith('#') or (len(paragraph.strip()) < 100 and paragraph.strip().isupper()):
                # If we have content in the current chunk, save it first
                if current_chunk.strip():
                    chunk_id = str(uuid.uuid4())
                    chunks.append((chunk_id, current_heading + current_chunk.strip()))

                # Start a new chunk with this heading
                current_heading = paragraph.strip() + "\n"
                current_chunk = ""
            elif len(current_chunk + paragraph) <= max_size:
                # Add paragraph to current chunk if it doesn't exceed max size
                current_chunk += paragraph + "\n\n"
            else:
                # If adding this paragraph would exceed max size, save current chunk
                if current_chunk.strip():
                    chunk_id = str(uuid.uuid4())
                    chunks.append((chunk_id, current_heading + current_chunk.strip()))

                # Start a new chunk
                current_heading = ""  # Reset heading for new chunk unless we find one
                current_chunk = paragraph + "\n\n"

                # If this single paragraph is too large, split it into sentences
                if len(current_chunk) > max_size:
                    sentence_chunks = TextChunker._split_large_paragraph(current_chunk, min_size, max_size)
                    for i, sentence_chunk in enumerate(sentence_chunks[:-1]):  # All but the last
                        chunk_id = str(uuid.uuid4())
                        chunks.append((chunk_id, current_heading + sentence_chunk.strip()))
                    current_chunk = sentence_chunks[-1]  # Keep the last part as current chunk

        # Add the final chunk if there's content
        if current_chunk.strip():
            chunk_id = str(uuid.uuid4())
            chunks.append((chunk_id, current_heading + current_chunk.strip()))

        # Merge small chunks with adjacent chunks if possible
        chunks = TextChunker._merge_small_chunks(chunks, min_size)

        return chunks

    @staticmethod
    def _split_large_paragraph(paragraph: str, min_size: int, max_size: int) -> List[str]:
        """
        Split a large paragraph into smaller chunks based on sentences
        """
        # Split by sentences
        sentences = re.split(r'(?<=[.!?]) +', paragraph)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk + sentence) <= max_size:
                current_chunk += sentence + " "
            else:
                if len(current_chunk) >= min_size or not chunks:
                    # If current chunk meets min size (or it's the first chunk), save it
                    chunks.append(current_chunk.strip())
                    current_chunk = sentence + " "
                else:
                    # If current chunk is too small, add it to the next chunk
                    current_chunk += sentence + " "

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    @staticmethod
    def _merge_small_chunks(chunks: List[Tuple[str, str]], min_size: int) -> List[Tuple[str, str]]:
        """
        Merge small chunks with adjacent chunks if possible
        """
        if len(chunks) <= 1:
            return chunks

        merged_chunks = []
        i = 0
        while i < len(chunks):
            chunk_id, chunk_text = chunks[i]

            # If this chunk is already large enough, keep it as is
            if len(chunk_text) >= min_size:
                merged_chunks.append((chunk_id, chunk_text))
                i += 1
            else:
                # This chunk is too small, try to merge with the next one
                if i + 1 < len(chunks):
                    next_id, next_text = chunks[i + 1]
                    # Check if merging would exceed max size
                    if len(chunk_text + " " + next_text) <= Config.CHUNK_SIZE_MAX:
                        # Merge with next chunk
                        merged_text = chunk_text + " " + next_text
                        merged_id = str(uuid.uuid4())  # Generate new ID for merged chunk
                        merged_chunks.append((merged_id, merged_text))
                        i += 2  # Skip both chunks since they're merged
                    else:
                        # Can't merge, keep both separate
                        merged_chunks.append((chunk_id, chunk_text))
                        merged_chunks.append((next_id, next_text))
                        i += 2
                else:
                    # Last chunk is small but there's nothing to merge with
                    merged_chunks.append((chunk_id, chunk_text))
                    i += 1

        return merged_chunks


def create_chunks_from_text(text: str, file_path: str, min_size: int = None, max_size: int = None) -> List[BookContentChunk]:
    """
    Create BookContentChunk objects from text, splitting into semantic chunks
    """
    if min_size is None:
        min_size = Config.CHUNK_SIZE_MIN
    if max_size is None:
        max_size = Config.CHUNK_SIZE_MAX

    chunker = TextChunker()
    chunk_tuples = chunker.chunk_text(text, min_size, max_size)

    chunks = []
    for chunk_id, chunk_text in chunk_tuples:
        # Extract heading if the chunk starts with one
        heading = None
        lines = chunk_text.split('\n')
        if lines and lines[0].strip().startswith('#'):
            heading = lines[0].strip()

        chunk = BookContentChunk(
            chunk_id=chunk_id,
            text=chunk_text,
            file_path=file_path,
            heading=heading
        )
        chunks.append(chunk)

    return chunks