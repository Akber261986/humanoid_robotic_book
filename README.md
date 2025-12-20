# Physical AI & Humanoid Robotics Book

This repository contains an educational book on Physical AI and Humanoid Robotics, focusing on practical applications of AI in physical systems with emphasis on embodied intelligence. Now enhanced with RAG + Chatbot capabilities!

## Overview

This educational resource provides a comprehensive introduction to Physical AI and Humanoid Robotics, covering:

- **Module 1**: Foundations of Physical AI and Humanoid Robotics
  - Introduction to Physical AI and embodied intelligence
  - Humanoid Robotics fundamentals and concepts
  - AI paradigms in robotics applications

- **Module 2**: Simulation Environments for Humanoid Robotics
  - ROS 2 setup and configuration for robotics
  - Gazebo simulation environment usage
  - NVIDIA Isaac Sim integration and best practices

- **Module 3**: Vision-Language Assistant (VLA) Integration
  - VLA concepts in robotics applications
  - Integration architectures and patterns
  - OpenVLA and Isaac Lab overview

- **Module 4**: Deployment and Real-World Applications
  - Transition from simulation to real-world deployment
  - Architecture considerations for deployment
  - ROS 2 deployment on physical hardware

## New RAG + Chatbot Feature

The book has been enhanced with an AI-powered assistant that allows users to:
- Ask natural-language questions about the book content
- Get explanations for selected text on any page
- Interact with a dedicated chat interface
- Access AI-powered search across all book modules
- Maintain persistent chat sessions with context

### Technology Stack for RAG Feature:
- **Vector Database**: Qdrant (local/cloud)
- **Embedding Model**: Google Gemini (embedding-001)
- **AI Model**: Google Gemini (gemini-1.5-flash)
- **Backend Framework**: FastAPI with Google Generative AI
- **Frontend Interface**: Streamlit chat application
- **Architecture**: Separated frontend and backend services

## Structure

- `docs/` - Contains all book content organized by modules
  - `module1/` - Foundations of Physical AI and Humanoid Robotics
  - `module2/` - Simulation Environments for Humanoid Robotics
  - `module3/` - Vision-Language Assistant Integration (Coming Soon)
  - `module4/` - Deployment and Real-World Applications (Coming Soon)
  - `examples/` - Reproducible code examples for each module

- `backend/` - Contains the RAG backend system
  - `src/` - Source code for the backend services
    - `api/` - FastAPI application with REST API endpoints
    - `models/` - Pydantic data models
    - `services/` - Core services (RAG, indexing, vector store, etc.)
    - `utils/` - Utility functions (chunking, parsing, etc.)
  - `requirements.txt` - Python dependencies
  - `Dockerfile` - Container configuration

- `frontend/` - Contains the Streamlit chat interface
  - `src/` - Source code for the frontend services
    - `services/` - API communication services
  - `app.py` - Main Streamlit application
  - `requirements.txt` - Python dependencies
  - `Dockerfile` - Container configuration

- `docusaurus.config.ts` - Docusaurus configuration for the book
- `sidebars.js` - Navigation structure for the book
- `SUMMARY.md` - Executive summary of the entire book
- `src/components/` - Custom React components including chat widget

## Quick Start

Follow these steps to set up and run the RAG-enabled chatbot for the Humanoid Robotics Book:

### Prerequisites
- Python 3.11+
- Docker (optional, for containerized deployment)
- Google Gemini API key
- Qdrant vector database (local or cloud)

### Setup Instructions

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd humanoid_robotic_book
   ```

2. **Set up the backend**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. **Configure environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env with your Gemini API key and Qdrant configuration
   ```

4. **Start Qdrant vector database** (if using local):
   ```bash
   docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
   ```

5. **Index the book content**:
   ```bash
   # From the backend directory
   python -c "
   from backend.src.services.indexer import Indexer
   indexer = Indexer()
   result = indexer.index_directory('../../docs')
   print('Indexing result:', result)
   "
   ```

6. **Start the backend API server**:
   ```bash
   # From the backend directory
   uvicorn backend.src.api.main:app --reload --host 0.0.0.0 --port 8000
   ```

7. **Set up the frontend** (in a new terminal):
   ```bash
   cd frontend  # Navigate to the frontend directory
   pip install -r requirements.txt
   ```

8. **Start the frontend Streamlit app**:
   ```bash
   # From the frontend directory
   streamlit run app.py
   ```

9. **Access the chatbot**:
   - Backend API: `http://localhost:8000`
   - Frontend UI: `http://localhost:8501`
   - API documentation: `http://localhost:8000/docs`

### Docker Deployment

To run both services using Docker:

1. **Build and run the backend**:
   ```bash
   cd backend
   docker build -t humanoid-robotics-backend .
   docker run -d -p 8000:8000 --env-file .env humanoid-robotics-backend
   ```

2. **Build and run the frontend**:
   ```bash
   cd frontend
   docker build -t humanoid-robotics-frontend .
   docker run -d -p 8501:8501 --env-file .env humanoid-robotics-frontend
   ```

## Target Audience

University students and early-career engineers with computer science/engineering background and introductory AI knowledge. The content balances theoretical foundations with practical implementation.

## Technical Requirements

- **Operating System**: Ubuntu 20.04+ (recommended)
- **Hardware**: NVIDIA GPU with Compute Capability 6.0+ (for Isaac Sim), 16GB+ RAM
- **Software**: Python 3.11, ROS 2 Humble Hawksbill, Gazebo Garden, Isaac Sim
- **Development Environment**: Linux command line proficiency required

## Learning Outcomes

Upon completing this book, readers will be able to:
1. Understand the principles of Physical AI and embodied intelligence
2. Set up and configure simulation environments (ROS 2, Gazebo, Isaac Sim)
3. Implement basic robotic control in simulation environments
4. Appreciate the challenges and opportunities in humanoid robotics
5. Prepare for advanced topics in VLA integration and real-world deployment

## Local Development

### Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create environment file:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and configuration
   ```

4. Index the book content into the vector database:
   ```bash
   # From the backend directory
   python -c "
   from backend.src.services.indexer import Indexer
   indexer = Indexer()
   result = indexer.index_directory('../docs')  # Adjust path as needed
   print('Indexing result:', result)
   "
   ```

5. Start the backend server:
   ```bash
   # From the backend directory
   uvicorn backend.src.api.main:app --reload --host 0.0.0.0 --port 8000
   ```

### Frontend Setup
1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Start the Streamlit frontend:
   ```bash
   # From the frontend directory
   streamlit run app.py
   ```

## Deployment

This book is built with [Docusaurus](https://docusaurus.io/) and is configured for GitHub Pages deployment. The RAG backend and Streamlit frontend are deployed as separate services to platforms like Railway, Heroku, or any container-compatible platform.

### GitHub Pages Deployment

The book content is configured to deploy automatically via GitHub Actions when changes are pushed to the main branch. The GitHub Actions workflow is defined in `.github/workflows/deploy.yml`.

To manually deploy, you can run:
```bash
GIT_USER=<your-username> CURRENT_BRANCH=main npm run deploy
```

The site will be deployed to: https://<your-username>.github.io/humanoid_robotic_book/

### Backend and Frontend Deployment

The RAG-enabled chatbot consists of two separate services:

1. **Backend API Service**: Deploy the backend using the provided Dockerfile to platforms like Railway, Heroku, or AWS:
   ```bash
   # Build and push the backend container
   cd backend
   docker build -t humanoid-robotics-backend .
   docker push <your-registry>/humanoid-robotics-backend
   ```

2. **Frontend Streamlit Service**: Deploy the frontend using the provided Dockerfile:
   ```bash
   # Build and push the frontend container
   cd frontend
   docker build -t humanoid-robotics-frontend .
   docker push <your-registry>/humanoid-robotics-frontend
   ```

### Docker Compose Deployment

Use the docker-compose.yml for easy local deployment of both services:

```bash
# Build and deploy both services with docker-compose
docker-compose up --build -d
```

## Testing the Complete Functionality

To test the complete RAG chatbot functionality after setup:

### Manual Testing

1. **Start the backend server**:
   ```bash
   cd backend
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

2. **In a separate terminal, run the test script**:
   ```bash
   python test_rag_chatbot.py
   ```

3. **Manual API testing**:
   - Visit `http://localhost:8000/docs` for interactive API documentation
   - Test the `/api/query-book` endpoint with sample queries
   - Check service status at `http://localhost:8000/status`

### End-to-End Testing

1. **Verify Qdrant integration**:
   ```bash
   # Check if your book content has been indexed
   curl "http://localhost:6333/collections"
   ```

2. **Test query functionality**:
   ```bash
   curl "http://localhost:8000/api/query-book?query=What%20is%20humanoid%20robotics"
   ```

3. **Test the floating chat widget**:
   - Start the Docusaurus site: `npm run start`
   - Open the book in your browser
   - Click the floating chat widget
   - Ask questions about the book content

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.