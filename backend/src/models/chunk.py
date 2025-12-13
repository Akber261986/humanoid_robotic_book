from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime


class BookContentChunk(BaseModel):
    """
    Represents a segment of book content that has been processed and stored in the vector database
    """
    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    text: str = Field(..., description="The actual text content of the chunk", min_length=1)
    file_path: str = Field(..., description="Path to the original source file")
    heading: Optional[str] = Field(None, description="Optional heading if the chunk starts with a section header")
    embedding: Optional[list] = Field(None, description="Vector representation of the text content")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata about the chunk")

    class Config:
        # Allow extra fields for flexibility
        extra = "allow"