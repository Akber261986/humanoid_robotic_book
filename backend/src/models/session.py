from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ChatSession(BaseModel):
    """
    Represents a persistent conversation context that maintains user query history and system responses
    """
    session_id: str = Field(..., description="Unique identifier for the session")
    created_at: datetime = Field(default_factory=datetime.now, description="When the session was created")
    last_accessed: datetime = Field(default_factory=datetime.now, description="When the session was last accessed")
    queries: Optional[List[str]] = Field(default_factory=list, description="List of query IDs in this session")

    class Config:
        # Allow extra fields for flexibility
        extra = "allow"
        # Enable datetime serialization
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }