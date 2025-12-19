# Humanoid Robotics RAG Chatbot API Documentation

## Overview
The Humanoid Robotics RAG Chatbot API provides endpoints for querying humanoid robotics book content using Retrieval-Augmented Generation (RAG). The system uses Google Gemini for AI responses and Qdrant for vector storage.

## Base URL
`http://localhost:8000`

## Endpoints

### GET /
**Description:** Root endpoint providing basic service information.
**Response:**
```json
{
  "message": "RAG Chatbot API",
  "version": "1.0.0",
  "endpoints": [
    "/api/query - Process user queries",
    "/api/index - Index book content",
    "/api/session - Manage chat sessions",
    "/health - Health check"
  ]
}
```

### GET /health
**Description:** Health check endpoint to verify service status.
**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T21:40:55.312921",
  "dependencies": {
    "qdrant": "connected",
    "gemini_api": "available"
  }
}
```

### GET /ready
**Description:** Readiness check endpoint including dependencies.
**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T21:40:55.312921",
  "dependencies": {
    "qdrant": "connected",
    "gemini_api": "available"
  }
}
```

### POST /api/session
**Description:** Create a new chat session.
**Request:** No body required.
**Response:**
```json
{
  "session_id": "9366bcd5-60d2-44df-90a3-15dd45da0be9",
  "created_at": "2025-12-16T21:41:23.930193"
}
```

### GET /api/session/{session_id}
**Description:** Get details about a specific session.
**Parameters:**
- `session_id`: The session identifier
**Response:**
```json
{
  "session_id": "9366bcd5-60d2-44df-90a3-15dd45da0be9",
  "created_at": "2025-12-16T21:41:23.930193",
  "last_accessed": "2025-12-16T21:42:10.123456",
  "query_count": 3
}
```

### POST /api/query
**Description:** Process a user query and return a RAG-enhanced response.
**Request:**
```json
{
  "query": "What is inverse kinematics in humanoid robotics?",
  "session_id": "9366bcd5-60d2-44df-90a3-15dd45da0be9"
}
```
**Request Fields:**
- `query`: The user's question about humanoid robotics (max 1000 characters)
- `session_id`: The chat session identifier

**Response:**
```json
{
  "response": "Inverse kinematics is the mathematical process of calculating the variable joint parameters needed to place the end of a robotic arm in a particular position and orientation...",
  "sources": [
    {
      "text": "Inverse kinematics is the mathematical process of calculating...",
      "file_path": "docs/module2/kinematics.md",
      "chunk_id": "chunk-12345"
    }
  ],
  "query_id": "uuid-of-query",
  "response_id": "uuid-of-response"
}
```

### POST /api/index
**Description:** Index book content from Markdown files into the vector database.
**Request:**
```json
{
  "docs_path": "../docs"
}
```
**Request Fields:**
- `docs_path`: Path to the directory containing Markdown files to index

**Response:**
```json
{
  "status": "completed",
  "chunks_indexed": 45,
  "files_processed": 5
}
```

### GET /api/chunk/{chunk_id}
**Description:** Retrieve a specific chunk by its ID for citation expansion.
**Parameters:**
- `chunk_id`: The unique identifier of the chunk
**Response:**
```json
{
  "chunk_id": "chunk-12345",
  "text": "Full text content of the chunk...",
  "file_path": "docs/module2/kinematics.md",
  "heading": "Inverse Kinematics",
  "metadata": {}
}
```

## Configuration
The API uses the following environment variables:
- `GEMINI_API_KEY`: Google Gemini API key (required)
- `QDRANT_URL`: URL for Qdrant server (default: http://localhost:6333)
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: book_vectors)
- `DEBUG`: Enable debug mode (default: False)
- `CHUNK_SIZE_MIN`: Minimum chunk size (default: 500)
- `CHUNK_SIZE_MAX`: Maximum chunk size (default: 800)
- `MAX_QUERY_LENGTH`: Maximum query length (default: 1000)
- `MAX_RESPONSE_WORDS`: Maximum response words (default: 300)
- `RETRIEVAL_TOP_K`: Number of top results to retrieve (default: 3)
- `RETRIEVAL_THRESHOLD`: Similarity threshold for retrieval (default: 0.7)

## Rate Limiting
The API implements rate limiting with 10 requests per 60 seconds per IP address.

## Error Handling
- 400: Bad Request - Invalid input parameters
- 404: Not Found - Resource not found
- 429: Too Many Requests - Rate limit exceeded
- 500: Internal Server Error - Server error occurred

## Deployment
The service can be deployed using Railway with the provided `railway.toml` configuration.