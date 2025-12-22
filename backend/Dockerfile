FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install dependencies
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY backend/main.py .
COPY backend/rag_simple.py .
COPY frontend_chat.html .
COPY tfidf_vectorizer.pkl .

# Copy the docs directory
COPY docs/ ./docs/

# Expose port
EXPOSE 8000

# Run the application from the correct directory
CMD ["python", "main.py"]