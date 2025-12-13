import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()


class Config:
    """
    Configuration class to manage application settings from environment variables
    """

    # Gemini API Configuration
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_EMBEDDING_MODEL: str = os.getenv("GEMINI_EMBEDDING_MODEL", "embedding-001")
    GEMINI_GENERATION_MODEL: str = os.getenv("GEMINI_GENERATION_MODEL", "gemini-1.5-flash")

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_vectors")

    # Application Configuration
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"
    CHUNK_SIZE_MIN: int = int(os.getenv("CHUNK_SIZE_MIN", "500"))
    CHUNK_SIZE_MAX: int = int(os.getenv("CHUNK_SIZE_MAX", "800"))
    MAX_QUERY_LENGTH: int = int(os.getenv("MAX_QUERY_LENGTH", "1000"))
    MAX_RESPONSE_WORDS: int = int(os.getenv("MAX_RESPONSE_WORDS", "300"))
    RETRIEVAL_TOP_K: int = int(os.getenv("RETRIEVAL_TOP_K", "3"))
    RETRIEVAL_THRESHOLD: float = float(os.getenv("RETRIEVAL_THRESHOLD", "0.7"))

    # Rate limiting
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "10"))
    RATE_LIMIT_WINDOW: int = int(os.getenv("RATE_LIMIT_WINDOW", "60"))  # in seconds

    @classmethod
    def validate_config(cls) -> bool:
        """
        Validate that required configuration values are present
        """
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        if cls.CHUNK_SIZE_MIN >= cls.CHUNK_SIZE_MAX:
            raise ValueError("CHUNK_SIZE_MIN must be less than CHUNK_SIZE_MAX")

        if cls.RETRIEVAL_THRESHOLD < 0 or cls.RETRIEVAL_THRESHOLD > 1:
            raise ValueError("RETRIEVAL_THRESHOLD must be between 0 and 1")

        return True