# Physical AI & Humanoid Robotics Book - RAG + Chatbot Enhancement

This repository contains the implementation of a RAG (Retrieval Augmented Generation) system for the Physical AI & Humanoid Robotics book, transforming it from a static book into an interactive knowledge base.

## Features

- **Interactive Q&A**: Users can ask natural-language questions about the book content
- **Selected Text Explanation**: Users can select text on any page and get explanations
- **Floating Chat Widget**: Always accessible chat interface on every page
- **Dedicated Chat Page**: Full-featured chat interface at `/chat`
- **Qdrant Vector Database**: Efficient storage and retrieval of book content
- **Google Gemini Integration**: Advanced AI for understanding and generating responses
- **Agentic SDK**: Advanced AI interactions for complex queries

## Architecture

The system consists of:

1. **Frontend**: Docusaurus-based documentation site with React components
   - Floating chat widget on all pages
   - Dedicated chat page with full interface
   - Text selection capabilities

2. **Backend**: FastAPI-based RAG system
   - Document ingestion pipeline
   - Vector storage with Qdrant
   - AI-powered query processing

3. **Integration**: Seamless connection between frontend and backend

## Technology Stack

- **Frontend**: Docusaurus v3, React, TypeScript
- **Backend**: FastAPI, Python
- **Vector Database**: Qdrant (cloud tier)
- **AI Models**: Google Gemini (embedding-001 and flash-2.5)
- **Backend Framework**: FastAPI with Google Generative AI
- **Deployment**: Docker, Docker Compose

## Setup and Installation

### Prerequisites

- Node.js 18+ (for Docusaurus)
- Python 3.8+ (for backend)
- Google Gemini API key
- Qdrant cloud account and API key
- Docker and Docker Compose (for deployment)

### Frontend Setup

1. Navigate to the project root:
```bash
cd humanoid_robotic_book
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

### Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Create environment file:
```bash
cp .env.example .env
# Edit .env with your API keys and configuration
```

4. Run the ingestion script:
```bash
python scripts/ingest_docs.py
```

5. Start the backend:
```bash
uvicorn api.main:app --reload --host 0.0.0.0 --port 8000
```

### Full Deployment

1. Set environment variables in `backend/.env`:
```bash
GEMINI_API_KEY=your_gemini_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
```

2. Run the deployment script:
```bash
./deploy.sh
```

## API Endpoints

- `GET /` - Health check
- `GET /health` - Health check with Qdrant status
- `POST /query` - Query the book content
- `POST /explain-selected` - Explain selected text from the book
- `GET /collections` - List available Qdrant collections

## How It Works

1. **Content Ingestion**: All Markdown files from the `/docs` directory are read, chunked, embedded using Google Gemini, and stored in Qdrant.

2. **Query Processing**: When a user asks a question, it's embedded using Google Gemini and searched against the vector database.

3. **Response Generation**: Relevant content is retrieved from Qdrant and passed to Google Gemini along with the query to generate a contextual response.

4. **Selected Text**: When users select text on any page, it can be sent to the backend for explanation.

## Environment Variables

### Backend
- `GEMINI_API_KEY` - Google Gemini API key
- `QDRANT_API_KEY` - Qdrant cloud API key
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_COLLECTION_NAME` - Collection name (default: humanoid_robotics_book)

## Development

To run both frontend and backend in development mode:

1. Start the backend:
```bash
cd backend
uvicorn api.main:app --reload
```

2. In a new terminal, start the frontend:
```bash
npm start
```

## Deployment

The system is designed for deployment to:
- Frontend: GitHub Pages (existing setup maintained)
- Backend: Vercel, Railway, or any container-compatible platform

The Docker and docker-compose setup facilitates deployment to any container orchestration platform.

## Files Structure

```
backend/
├── api/                 # FastAPI backend
├── scripts/            # Ingestion and utility scripts
├── requirements.txt    # Python dependencies
├── Dockerfile         # Backend container configuration
└── .env.example       # Environment variables template
src/
├── components/         # React components (FloatingChatWidget)
├── pages/chat/        # Dedicated chat page
└── theme/Root.tsx     # Global theme wrapper
```

## Testing

To test the integration:

1. Start both frontend and backend
2. Navigate to the book site
3. Use the floating chat widget or visit the `/chat` page
4. Ask questions about the book content
5. Select text on any page to get explanations

## Troubleshooting

- Ensure all environment variables are properly set
- Check that the ingestion script ran successfully
- Verify Qdrant connection and collection exists
- Check CORS settings if deploying to different domains

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

MIT License - see LICENSE file for details.