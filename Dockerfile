# Use an official Python runtime as a parent image
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1
ENV PYTHONWARNINGS="ignore::UserWarning:fastembed.embedding"

# Set the working directory
WORKDIR /app

# Copy the requirements file
COPY requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Pre-download the embedding model during build
RUN python -c "from fastembed import TextEmbedding; TextEmbedding(model_name='BAAI/bge-small-en-v1.5')"

# Copy the entire backend directory to the working directory
COPY backend/ .

# Expose port (will be overridden by ${PORT} environment variable)
EXPOSE 8000

# Create a startup script that starts the server immediately to avoid Railway timeouts and suppresses fastembed warnings
RUN echo '#!/bin/bash\n\
echo "Starting server..."\n\
export PYTHONWARNINGS="ignore::UserWarning:fastembed.embedding"\n\
uvicorn api.main:app --host 0.0.0.0 --port ${PORT:-8000}' > startup.sh && \
chmod +x startup.sh

# Run the startup script
CMD ["./startup.sh"]