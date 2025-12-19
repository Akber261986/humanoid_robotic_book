import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Config:
    # Gemini API Configuration
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
    GEMINI_EMBEDDING_MODEL = os.getenv("GEMINI_EMBEDDING_MODEL", "embedding-001")
    GEMINI_GENERATION_MODEL = os.getenv("GEMINI_GENERATION_MODEL", "gemini-1.5-flash")

    # Qdrant Configuration
    QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

    # Application Configuration
    DOCS_PATH = os.getenv("DOCS_PATH", "../docs")
    CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "1000"))
    OVERLAP_SIZE = int(os.getenv("OVERLAP_SIZE", "100"))
    TOP_K = int(os.getenv("TOP_K", "3"))
    SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))

    @classmethod
    def validate(cls):
        """Validate that required configuration values are present"""
        if not cls.GEMINI_API_KEY:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        if not cls.QDRANT_URL:
            raise ValueError("QDRANT_URL environment variable is required")

        return True