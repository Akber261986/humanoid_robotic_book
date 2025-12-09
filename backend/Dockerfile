# Use an official Python runtime as a parent image
FROM python:3.11-slim

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Set the working directory
WORKDIR /app

# Copy the requirements file
COPY requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY . .

# Expose port 8000
EXPOSE 8000

# Create a startup script that runs ingestion first, then starts the server
RUN echo '#!/bin/bash\n\
echo "Starting ingestion process..."\n\
python scripts/ingest_docs.py\n\
echo "Ingestion completed. Starting server..."\n\
uvicorn api.main:app --host 0.0.0.0 --port ${PORT:-8000}' > startup.sh && \
chmod +x startup.sh

# Run the startup script
CMD ["./startup.sh"]