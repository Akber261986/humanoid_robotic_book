# Railway Deployment Guide

This guide explains how to deploy the RAG Chatbot application on Railway.

## Prerequisites

- A Railway account (sign up at [railway.app](https://railway.app))
- Your Google Gemini API key
- Git installed on your local machine

## Deployment Steps

### 1. Deploy via Railway Button (Recommended)

[![Deploy on Railway](https://railway.app/button.svg)](https://railway.app/new/template?template=https://github.com/your-repo/rag-chatbot)

### 2. Manual Deployment

1. Install the Railway CLI:
   ```bash
   npm install -g @railway/cli
   ```

2. Login to Railway:
   ```bash
   railway login
   ```

3. Create a new project:
   ```bash
   railway init
   ```

4. Link your repository:
   ```bash
   railway link
   ```

5. Set environment variables:
   ```bash
   railway variables set GEMINI_API_KEY=your_gemini_api_key_here
   ```

6. Deploy the application:
   ```bash
   railway up
   ```

## Environment Variables

The following environment variables need to be set in your Railway dashboard:

- `GEMINI_API_KEY`: Your Google Gemini API key
- `PORT`: Port number for the application (default: 8000)
- `PYTHON_ENV`: Environment type (default: production)

## Configuration Details

The application uses the following configuration:

- **Backend**: Python FastAPI application
- **Database**: Qdrant vector database (can be deployed as a separate service)
- **Frontend**: Streamlit interface

## Troubleshooting

### Application Sleeping

If your application goes to sleep (common on Railway's free tier):

1. Check your Railway plan limits
2. Consider upgrading to a paid plan for continuous deployment
3. Set up a uptime monitoring service to ping your app periodically

### API Key Issues

If you encounter API key errors:

1. Verify the `GEMINI_API_KEY` variable is set correctly
2. Check that the API key has the necessary permissions
3. Ensure the API key hasn't expired

### Qdrant Connection Issues

If the application can't connect to Qdrant:

1. Verify Qdrant is running and accessible
2. Check the `QDRANT_HOST` and `QDRANT_PORT` variables
3. Ensure proper network connectivity between services

## Scaling

To scale your application:

1. In the Railway dashboard, navigate to your project
2. Adjust the resources allocated to your service
3. Consider adding a separate Qdrant service for production use