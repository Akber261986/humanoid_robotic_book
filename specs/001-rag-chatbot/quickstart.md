# Quickstart Guide: RAG-Enabled Gemini Chatbot for Humanoid Robotics Book

**Feature**: 001-rag-chatbot
**Date**: 2025-12-11
**Status**: Completed

## Overview

This guide provides step-by-step instructions to set up, configure, and run the RAG-enabled Gemini Chatbot for Humanoid Robotics Book. The system is built with separate frontend and backend components as required by the constitution.

## Prerequisites

- Python 3.11+
- Docker (for running Qdrant locally)
- Google Generative AI API key
- Git for cloning the repository

## Setup Instructions

### 1. Clone the Repository

```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Set up Backend

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Set up Frontend

```bash
# Navigate to frontend directory
cd frontend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Create a `.env` file in the root directory with the following content:

```env
GEMINI_API_KEY=your_google_generative_ai_api_key_here
QDRANT_URL=http://localhost:6333
```

### 5. Start Qdrant Vector Database

```bash
# Run Qdrant in Docker
docker run -p 6333:6333 qdrant/qdrant
```

### 6. Index Book Content

```bash
# From the backend directory
cd backend
source venv/bin/activate
python src/services/indexer.py
```

This will process the Markdown files in the `docs/` directory and store them in the Qdrant vector database.

### 7. Start Backend API Server

```bash
# From the backend directory
cd backend
source venv/bin/activate
python -m src.api.main
```

### 8. Start Frontend Application

```bash
# From the frontend directory
cd frontend
source venv/bin/activate
streamlit run app.py
```

## Usage

1. Open your browser and navigate to the Streamlit application (typically http://localhost:8501)
2. Enter your question about humanoid robotics in the chat interface
3. The system will retrieve relevant book content and generate a response with citations
4. Click on citations to view the original book sections

## Testing

### Backend Tests

```bash
# From the backend directory
cd backend
python -m pytest tests/
```

### Manual Testing

1. Test that queries return relevant responses
2. Verify that citations link to correct book sections
3. Confirm that chat history is maintained during the session
4. Test error handling when the book database is empty

## Deployment

### Local Deployment

For local deployment, follow the setup instructions above.

### Railway Deployment

1. Connect your Railway account
2. Create new project and link to your repository
3. Add environment variables in Railway dashboard
4. Deploy the backend and frontend as separate services

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your GEMINI_API_KEY is valid and has proper permissions
2. **Qdrant Connection**: Verify Qdrant is running on http://localhost:6333
3. **Indexing Failure**: Check that the docs/ directory contains valid Markdown files
4. **Streamlit Not Starting**: Ensure all frontend dependencies are installed

### Logs

- Backend logs: Check console output from backend server
- Frontend logs: Check console output from Streamlit
- Qdrant logs: Available through Docker logs