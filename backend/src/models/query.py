from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class UserQuery(BaseModel):
    """
    Represents a text input from the user seeking information about humanoid robotics topics
    """
    query_id: str = Field(..., description="Unique identifier for the query")
    text: str = Field(..., description="The actual query text from the user", max_length=1000)
    timestamp: datetime = Field(default_factory=datetime.now, description="When the query was submitted")
    session_id: str = Field(..., description="Identifier for the chat session")

    class Config:
        # Allow extra fields for flexibility
        extra = "allow"
        # Enable datetime serialization
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }