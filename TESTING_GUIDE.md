# Testing the Complete RAG Chatbot Functionality

This guide provides comprehensive instructions for testing the complete RAG chatbot system.

## Prerequisites

Before testing the complete system, ensure:

1. **Docker is installed and running**
2. **Qdrant container is running**:
   ```bash
   docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
   ```

3. **Backend dependencies installed**:
   ```bash
   cd backend && pip install -r requirements.txt
   ```

4. **Frontend dependencies installed**:
   ```bash
   cd frontend && pip install -r requirements.txt
   ```

5. **Environment files configured**:
   - `backend/.env` with valid GEMINI_API_KEY
   - `frontend/.env` with correct BACKEND_URL

6. **Book content indexed** (see index_book_content.py)

## Complete System Test Steps

### Step 1: Start Qdrant
```bash
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
```

Verify it's running:
```bash
curl http://localhost:6333
```

### Step 2: Index Book Content
```bash
cd backend
python ../index_book_content.py
```

### Step 3: Start Backend Server
```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### Step 4: Start Frontend App
In a new terminal:
```bash
cd frontend
streamlit run app.py
```

### Step 5: Verify All Components

#### Backend Health Check:
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

#### Frontend Access:
Open http://localhost:8501 in your browser

## Functional Tests

### Test 1: Basic Query
1. Open the frontend app
2. Ask a simple question like "What is humanoid robotics?"
3. Verify you get a response with sources
4. Check that sources are properly cited

### Test 2: Session Management
1. Ask multiple questions in the same session
2. Verify session ID remains consistent
3. Check session details in the sidebar
4. Test clearing the chat

### Test 3: API Endpoints
Test each endpoint manually or with a tool like Postman:

#### Create Session:
```bash
curl -X POST http://localhost:8000/api/session
```

#### Query Endpoint:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main topics covered?",
    "session_id": "your-session-id"
  }'
```

#### Get Session Details:
```bash
curl http://localhost:8000/api/session/your-session-id
```

### Test 4: Error Handling
1. Test with empty query
2. Test with very long query (>1000 chars)
3. Test with invalid session ID
4. Verify proper error messages

## Performance Tests

### Response Time
- Measure time from query submission to response
- Expected: < 5 seconds for typical queries
- Check that citations don't significantly impact response time

### Concurrency
- Test multiple simultaneous users/requests
- Verify rate limiting works (10 requests per 60 seconds by default)

## Integration Tests

### End-to-End Flow
1. Start a new session
2. Ask 3-5 related questions
3. Verify context is maintained
4. Check that sources are relevant
5. Verify citations point to correct documents

### Data Flow Verification
1. Verify content was properly chunked during indexing
2. Check that embeddings were created
3. Confirm retrieval returns relevant chunks
4. Validate that RAG pipeline generates accurate responses

## Common Issues and Solutions

### Issue: Qdrant Connection Failure
**Symptoms**: Backend fails to start or API returns 500 errors
**Solution**:
- Verify Docker is running
- Check `docker ps` shows qdrant-container
- Confirm QDRANT_URL in .env matches Qdrant container port

### Issue: Gemini API Key Error
**Symptoms**: "Invalid API key" or 401 errors
**Solution**:
- Verify GEMINI_API_KEY in backend/.env
- Ensure the API key has proper permissions
- Check the key hasn't expired

### Issue: Slow Responses
**Symptoms**: Queries taking >10 seconds
**Solution**:
- Check internet connection
- Verify Gemini API is responding
- Consider reducing RETRIEVAL_TOP_K in config

### Issue: No Relevant Results
**Symptoms**: Responses say "couldn't find relevant information"
**Solution**:
- Verify content was properly indexed
- Check that docs directory has content
- Adjust RETRIEVAL_THRESHOLD if too strict

## Verification Checklist

- [ ] Qdrant is running and accessible
- [ ] Backend server starts without errors
- [ ] Frontend app loads successfully
- [ ] Health check returns healthy status
- [ ] Indexing completes successfully
- [ ] Session creation works
- [ ] Query processing returns valid responses
- [ ] Source citations are provided
- [ ] Frontend displays responses correctly
- [ ] Error handling works properly
- [ ] Rate limiting is enforced
- [ ] Session management functions correctly

## Expected Results

When fully functional, the system should:
- Answer questions based on the book content
- Provide relevant source citations
- Maintain conversation context
- Handle errors gracefully
- Respect rate limits
- Process queries in under 5 seconds
- Maintain sessions across multiple queries

## Troubleshooting Commands

### Check all running containers:
```bash
docker ps
```

### Check backend logs:
```bash
# While running uvicorn, check the terminal output
```

### Check Qdrant logs:
```bash
docker logs qdrant-container
```

### Verify API connectivity:
```bash
curl http://localhost:8000/docs
```

### Test indexing:
```bash
curl -X POST http://localhost:8000/api/index \
  -H "Content-Type: application/json" \
  -d '{"docs_path": "../docs"}'
```