# Starting the Frontend Streamlit App

This guide explains how to start the frontend Streamlit application for the RAG chatbot.

## Prerequisites

- Backend API server running (see START_BACKEND.md)
- Python 3.11+ with dependencies installed
- Streamlit and requests libraries available

## Starting the Frontend

### Method 1: Using Streamlit (Development)

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Start the Streamlit app:
```bash
streamlit run app.py
```

3. The application will be available at:
   - http://localhost:8501

### Method 2: Using Python

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Run the app directly:
```bash
python app.py
```

### Method 3: Using Docker (Production)

1. Build the Docker image:
```bash
docker build -t humanoid-robotics-frontend .
```

2. Run the container:
```bash
docker run -d -p 8501:8501 --env-file .env humanoid-robotics-frontend
```

## Configuration

### Environment Variables

Make sure your `frontend/.env` file contains:

```env
BACKEND_URL=http://localhost:8000
```

If your backend is running on a different host/port, update accordingly.

## Features

The frontend provides:

- **Chat Interface**: Interactive conversation with the RAG bot
- **Session Management**: Persistent chat sessions with unique IDs
- **Source Citations**: Expandable sections showing source documents
- **Health Monitoring**: Real-time status of backend services
- **Clear Chat**: Button to reset the conversation
- **Session Details**: Information about the current session

## Troubleshooting

### Common Issues:

1. **Cannot connect to backend**: Verify the backend API server is running
2. **Streamlit not found**: Install Streamlit: `pip install streamlit`
3. **Port already in use**: Streamlit will ask to use a different port
4. **Slow responses**: May be waiting for API calls to complete

### Testing Connection:

Before using the full app, verify the backend connection:
```bash
curl http://localhost:8000/health
```

### Custom Port:

To run Streamlit on a different port:
```bash
streamlit run app.py --server.port 8502
```

## Using the Application

1. **Initial Load**: The app will automatically create a session
2. **Asking Questions**: Type your question in the chat input
3. **Viewing Sources**: Click on "Sources Used" expanders to see citations
4. **Session Info**: Check the sidebar for session details
5. **Clearing Chat**: Use the "Clear Chat" button to start fresh

## Backend URL Configuration

If your backend is running on a different server:

1. Update `frontend/.env`:
```env
BACKEND_URL=http://your-backend-server:8000
```

2. Or update the URL directly in `frontend/app.py`:
```python
st.session_state.chat_service = ChatService(base_url="http://your-backend:8000")
```

## Security Considerations

- In production, use HTTPS for both frontend and backend
- Implement proper authentication if needed
- Set appropriate CORS policies
- Validate all user inputs