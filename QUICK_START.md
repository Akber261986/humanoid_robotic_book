# Quick Start Guide - Humanoid Robotics RAG Chatbot

## Overview
This guide will help you quickly set up and run the Humanoid Robotics RAG Chatbot that allows you to ask questions about humanoid robotics based on book content.

## Prerequisites
- Python 3.11+
- Docker (for Qdrant vector database)
- Google Gemini API Key

## Quick Setup Steps

### 1. Clone and Navigate
```bash
git clone <repository-url>
cd humanoid_robotic_book/backend
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Set Up Environment Variables
Create a `.env` file in the `backend` directory:
```env
GEMINI_API_KEY=your_google_gemini_api_key_here
```

### 5. Start Qdrant Database
```bash
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
```

### 6. Start the Backend Server
```bash
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8000
```

### 7. Start the Frontend
```bash
cd ../frontend
streamlit run app.py
```

## Using the Chatbot

1. Open your browser to `http://localhost:8501`
2. The chatbot will automatically create a session
3. Ask questions about humanoid robotics, such as:
   - "What are the main components of a humanoid robot?"
   - "Explain inverse kinematics in humanoid robotics"
   - "How do humanoid robots maintain balance?"

## API Usage (Alternative to Web Interface)

### Index the Book Content
```bash
curl -X POST http://localhost:8000/api/index \
  -H "Content-Type: application/json" \
  -d '{"docs_path": "../docs"}'
```

### Ask Questions via API
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is inverse kinematics?", "session_id": "session-id-from-step-1"}'
```

## Next Steps
- Explore the full API documentation in [API_DOCUMENTATION.md](API_DOCUMENTATION.md)
- Check the complete setup guide in [SETUP_GUIDE.md](SETUP_GUIDE.md)
- Review the project architecture and features in the main documentation

## Troubleshooting
- If you get Qdrant connection errors, ensure the Docker container is running
- If you get Gemini API errors, verify your API key is correct
- Check that all required ports (8000 for backend, 8501 for frontend, 6333 for Qdrant) are available