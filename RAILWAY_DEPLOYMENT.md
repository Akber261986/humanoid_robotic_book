# Railway Deployment Configuration

This project is configured for deployment on Railway. The following changes were made to enable successful deployment:

## Files Added/Modified:
- `requirements.txt` - Copied from `backend/` directory to project root so Railway can detect the Python project
- `Dockerfile` - Created in project root with proper build instructions for Railway

## Deployment Steps:
1. Connect your GitHub repository to Railway
2. Railway will automatically detect the `requirements.txt` file and use the `Dockerfile` for building
3. Set the following environment variables in Railway:
   - `GEMINI_API_KEY` - Google Gemini API Key
   - `QDRANT_API_KEY` - Qdrant API Key
   - `QDRANT_URL` - Qdrant server URL
   - `QDRANT_COLLECTION_NAME` - Qdrant collection name (default: humanoid_robotics_book)
   - `OPENAI_API_KEY` - OpenAI API Key (if needed)

## Build Process:
- The Dockerfile installs dependencies from requirements.txt
- Copies backend directory contents to maintain proper application structure
- Runs the ingestion script on startup before starting the server
- Uses the PORT environment variable provided by Railway

## Notes:
- The application will run `python scripts/ingest_docs.py` first to set up the vector database
- Then it starts the FastAPI server with uvicorn on the port specified by the PORT environment variable