# Humanoid Robotics Book - RAG Chatbot Setup Guide

## Prerequisites

Before running the application, ensure you have:

1. **Python 3.11+** installed
2. **Docker** (for Qdrant vector database)
3. **Google Gemini API Key** (for AI functionality)

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd humanoid_robotic_book
```

### 2. Set up Python Environment
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Set up Environment Variables
Create a `.env` file in the `backend` directory with the following content:

```env
GEMINI_API_KEY=your_google_gemini_api_key_here
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION_NAME=book_vectors
DEBUG=False
CHUNK_SIZE_MIN=500
CHUNK_SIZE_MAX=800
MAX_QUERY_LENGTH=1000
MAX_RESPONSE_WORDS=300
RETRIEVAL_TOP_K=3
RETRIEVAL_THRESHOLD=0.7
RATE_LIMIT_REQUESTS=10
RATE_LIMIT_WINDOW=60
```

> **Note:** Get your Gemini API key from [Google AI Studio](https://aistudio.google.com/)

## Running the Application

### 1. Start Qdrant Vector Database
```bash
# Option A: Using Docker (recommended)
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant

# Option B: Using Docker Compose (if available)
docker-compose up -d qdrant
```

### 2. Start the Backend API Server
```bash
cd backend
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8000
# or
python run_server.py
```

### 3. Start the Frontend (Streamlit)
```bash
cd frontend
streamlit run app.py
```

## Using the Application

### 1. Index the Book Content
Before asking questions, you need to index the book content:

```bash
curl -X POST http://localhost:8000/api/index \
  -H "Content-Type: application/json" \
  -d '{"docs_path": "../docs"}'
```

This will process all Markdown files in the `docs/` directory and store their vector representations in Qdrant.

### 2. Create a Session
```bash
curl -X POST http://localhost:8000/api/session
```

### 3. Ask Questions
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is inverse kinematics in humanoid robotics?", "session_id": "your-session-id"}'
```

### 4. Using the Web Interface
Alternatively, use the Streamlit web interface at `http://localhost:8501` to interact with the chatbot.

## API Endpoints

For detailed API documentation, see [API_DOCUMENTATION.md](API_DOCUMENTATION.md)

## Troubleshooting

### Qdrant Connection Issues
- Ensure Qdrant is running on `http://localhost:6333`
- Check Docker container status: `docker ps | grep qdrant`
- If Qdrant fails to start, try: `docker restart qdrant-container`

### Gemini API Issues
- Verify your API key is correct in the `.env` file
- Check if the API key has the necessary permissions
- Ensure you're within your API usage limits

### Indexing Issues
- Verify the `docs_path` points to a valid directory with Markdown files
- Check that the directory contains `.md` files
- Ensure the application has read permissions for the directory

## Development

### Running Tests
```bash
cd backend
python -m pytest tests/
```

### Environment Setup for Development
```bash
pip install -r requirements.txt
pip install pytest black flake8
```

## Deployment

### Railway Deployment
The application is configured for Railway deployment. The `railway.toml` file contains the necessary configuration.

To deploy:
1. Install the Railway CLI
2. Run `railway login`
3. Run `railway up`
4. Add your `GEMINI_API_KEY` as a secret in the Railway dashboard

### Docker Deployment
A `Dockerfile` is provided in the `backend` directory for containerized deployment.

## Project Structure
```
humanoid_robotic_book/
├── backend/                 # FastAPI backend
│   ├── src/
│   │   ├── api/            # API endpoints
│   │   ├── services/       # Business logic
│   │   └── models/         # Data models
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/               # Streamlit frontend
│   └── app.py
├── docs/                   # Book content in Markdown
│   ├── module1/
│   ├── module2/
│   └── ...
├── .env.example           # Example environment variables
└── API_DOCUMENTATION.md   # API documentation
```

## Health Checks
- Health: `GET http://localhost:8000/health`
- Readiness: `GET http://localhost:8000/ready`

## Rate Limiting
The API implements rate limiting with 10 requests per 60 seconds per IP address.