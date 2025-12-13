from pydantic import BaseModel, Field
from typing import List, Dict
from datetime import datetime


class SourceChunk(BaseModel):
    """
    Represents a source chunk used in a response
    """
    text: str = Field(..., description="The text content of the source chunk")
    file_path: str = Field(..., description="Path to the original source file")
    chunk_id: str = Field(..., description="Unique identifier of the source chunk")


class ChatResponse(BaseModel):
    """
    Represents the system's response to a user query, including the generated text and source citations
    """
    response_id: str = Field(..., description="Unique identifier for the response")
    query_id: str = Field(..., description="Reference to the corresponding query")
    text: str = Field(..., description="The generated response text", max_length=3000)  # Approximate 300 words
    sources: List[SourceChunk] = Field(..., description="List of source chunks used to generate the response")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the response was generated")

    class Config:
        # Allow extra fields for flexibility
        extra = "allow"
        # Enable datetime serialization
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }