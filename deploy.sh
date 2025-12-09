#!/bin/bash

# Deployment script for Physical AI & Humanoid Robotics Book RAG system

set -e  # Exit on any error

echo "Starting deployment of Physical AI & Humanoid Robotics Book RAG system..."

# Check if environment variables are set
if [ -z "$GEMINI_API_KEY" ] || [ -z "$QDRANT_API_KEY" ] || [ -z "$QDRANT_URL" ]; then
    echo "Error: Required environment variables are not set."
    echo "Please set GEMINI_API_KEY, QDRANT_API_KEY, and QDRANT_URL"
    exit 1
fi

echo "Environment variables are set. Proceeding with deployment..."

# Run the ingestion script to populate the vector database
echo "Running ingestion script to populate Qdrant vector database..."
cd backend
python scripts/ingest_docs.py
cd ..

echo "Ingestion completed successfully."

# Build and start the services using docker-compose
echo "Building and starting services..."
docker-compose up -d

echo "Services are running. Backend is available at http://localhost:8000"
echo "Qdrant dashboard is available at http://localhost:6333"

echo "Deployment completed successfully!"
echo ""
echo "To verify the deployment:"
echo "1. Check backend health: curl http://localhost:8000/health"
echo "2. Visit the book site: https://akber261986.github.io/humanoid_robotic_book/"
echo "3. Use the 'Ask the Book' feature in the navbar or sidebar"