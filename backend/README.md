# Physical AI & Humanoid Robotics Book - RAG Backend

This backend provides a RAG (Retrieval Augmented Generation) system for the Physical AI & Humanoid Robotics book.

## Features

- FastAPI-based backend with Google Gemini integration
- Qdrant vector database for document storage and retrieval
- Google Gemini Flash 2.5 for advanced AI interactions
- Endpoints for querying book content and explaining selected text

## Prerequisites

- Python 3.8+
- Google Gemini API key
- Qdrant cloud account and API key

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Create a `.env` file based on `.env.example`:
```bash
cp .env.example .env
# Edit .env with your actual API keys and configuration
```

3. Run the ingestion script to populate the vector database:
```bash
python scripts/ingest_docs.py
```

4. Start the backend server:
```bash
uvicorn api.main:app --reload --host 0.0.0.0 --port 8000
```

## API Endpoints

- `GET /` - Health check
- `GET /health` - Health check with Qdrant status
- `POST /query` - Query the book content
- `POST /explain-selected` - Explain selected text from the book
- `GET /collections` - List available Qdrant collections

## Environment Variables

- `GEMINI_API_KEY` - Your Google Gemini API key
- `QDRANT_API_KEY` - Your Qdrant API key
- `QDRANT_URL` - Your Qdrant cluster URL
- `QDRANT_COLLECTION_NAME` - Name of the Qdrant collection (default: humanoid_robotics_book)

## Development

To run in development mode with auto-reload:
```bash
python -m uvicorn api.main:app --reload
```