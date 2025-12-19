# Starting the Backend API Server

This guide explains how to start the backend API server for the RAG chatbot.

## Prerequisites

- Docker and Qdrant running (see DOCKER_SETUP_GUIDE.md)
- Python 3.11+ with dependencies installed
- GEMINI_API_KEY set in `backend/.env`

## Starting the Server

### Method 1: Using Uvicorn (Development)

1. Navigate to the backend directory:
```bash
cd backend
```

2. Start the server:
```bash
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

3. The API will be available at:
   - http://localhost:8000
   - API docs: http://localhost:8000/docs
   - Health check: http://localhost:8000/health

### Method 2: Using Python Directly

1. Navigate to the backend directory:
```bash
cd backend
```

2. Run the main module:
```bash
python -m src.api.main
```

### Method 3: Using Docker (Production)

1. Build the Docker image:
```bash
docker build -t humanoid-robotics-backend .
```

2. Run the container (make sure Qdrant is accessible):
```bash
docker run -d -p 8000:8000 --env-file .env --network host humanoid-robotics-backend
```

## API Endpoints

Once running, the following endpoints will be available:

- `POST /api/query` - Process user queries with RAG
- `POST /api/index` - Index documents into vector database
- `POST /api/session` - Create new chat sessions
- `GET /api/session/{session_id}` - Get session details
- `GET /api/chunk/{chunk_id}` - Retrieve specific content chunk
- `GET /health` - Health check endpoint
- `GET /docs` - Interactive API documentation

## Troubleshooting

### Common Issues:

1. **Connection to Qdrant fails**: Verify Qdrant is running and accessible
2. **API key missing**: Ensure GEMINI_API_KEY is set in .env
3. **Port already in use**: Use a different port number
4. **Memory issues**: Ensure sufficient RAM is available

### Health Check:

Before using the API, verify the service is healthy:
```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-12T18:15:30.123456",
  "dependencies": {
    "qdrant": "connected",
    "gemini_api": "available"
  }
}
```

## Environment Variables

Make sure your `backend/.env` file contains:

```env
GEMINI_API_KEY=your_api_key_here
GEMINI_EMBEDDING_MODEL=embedding-001
GEMINI_GENERATION_MODEL=gemini-1.5-flash
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION_NAME=book_vectors
CHUNK_SIZE_MIN=500
CHUNK_SIZE_MAX=800
MAX_QUERY_LENGTH=1000
MAX_RESPONSE_WORDS=300
RETRIEVAL_TOP_K=3
RETRIEVAL_THRESHOLD=0.7
```

## Testing the API

Once the server is running, you can test it with curl:

```bash
curl -X POST http://localhost:8000/api/session
```

Or test a query (after indexing content):
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is humanoid robotics?",
    "session_id": "your-session-id"
  }'
```