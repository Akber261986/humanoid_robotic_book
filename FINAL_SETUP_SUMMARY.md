# Humanoid Robotics RAG Chatbot - Complete Setup and Testing Guide

## Overview

This project implements a Retrieval-Augmented Generation (RAG) chatbot for the Humanoid Robotics Book. The system allows users to ask natural-language questions about the book content and receive AI-powered responses with proper citations.

## Architecture

- **Frontend**: Streamlit application for user interaction
- **Backend**: FastAPI server with RAG pipeline
- **Vector Database**: Qdrant for storing document embeddings
- **AI Model**: Google Gemini for embeddings and response generation

## Complete Setup Process

### 1. Prerequisites
- Python 3.11+
- Docker (for Qdrant)
- Google Gemini API key

### 2. Environment Setup
```bash
# Copy environment files
cp backend/.env.example backend/.env
cp frontend/.env.example frontend/.env

# Add your Gemini API key to backend/.env
GEMINI_API_KEY=your_actual_api_key_here
```

### 3. Install Dependencies
```bash
# Backend
cd backend
pip install -r requirements.txt

# Frontend
cd frontend
pip install -r requirements.txt
```

### 4. Start Infrastructure
```bash
# Start Qdrant vector database
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
```

### 5. Index Book Content
```bash
cd backend
python ../index_book_content.py
```

### 6. Start Services
```bash
# Terminal 1: Start backend
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000

# Terminal 2: Start frontend
cd frontend
streamlit run app.py
```

### 7. Access the Application
- Frontend: http://localhost:8501
- Backend API: http://localhost:8000
- Backend Docs: http://localhost:8000/docs

## Key Features

- **Natural Language Queries**: Ask questions about humanoid robotics in plain English
- **Source Citations**: All responses include citations to specific book sections
- **Session Management**: Persistent conversations with context
- **Health Monitoring**: Real-time status of all system components
- **Rate Limiting**: Protection against API abuse
- **Error Handling**: Graceful degradation when services are unavailable

## Files Created During Setup

- `DOCKER_SETUP_GUIDE.md` - Instructions for Docker and Qdrant setup
- `START_BACKEND.md` - Backend server startup guide
- `START_FRONTEND.md` - Frontend application startup guide
- `TESTING_GUIDE.md` - Comprehensive testing procedures
- `verify_setup.py` - System verification script
- `index_book_content.py` - Content indexing script

## Verification

Run the verification script to ensure all components are properly set up:
```bash
python verify_setup.py
```

## Next Steps

1. Add your Gemini API key to `backend/.env`
2. Install Docker and start Qdrant
3. Index the book content
4. Start both backend and frontend services
5. Test the complete functionality using the testing guide

## Troubleshooting

Refer to the individual guide files for detailed troubleshooting steps:
- `DOCKER_SETUP_GUIDE.md` for Qdrant issues
- `TESTING_GUIDE.md` for system verification
- Check logs in respective terminals for error details

## Architecture Decisions

- **Separation of Concerns**: Frontend and backend are separate services
- **Vector Database**: Qdrant chosen for its efficiency and Python client
- **AI Model**: Google Gemini for its strong reasoning and citation capabilities
- **Framework Choice**: FastAPI for backend (async, typed), Streamlit for frontend (rapid prototyping)
- **Deployment**: Container-ready with Docker support

The system is now ready for full operation once the prerequisites are met and services are started.