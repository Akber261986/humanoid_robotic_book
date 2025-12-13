# GitHub Actions + Railway Deployment

This document explains how to set up automatic deployment to Railway using GitHub Actions.

## Prerequisites

1. Railway project set up with your application
2. Railway CLI installed locally
3. GitHub repository connected to your Railway project

## Setup Instructions

### 1. Get Your Railway Token

1. Log in to Railway from the CLI:
   ```bash
   railway login
   ```

2. Get your API token:
   ```bash
   railway token
   ```

### 2. Add the Token to GitHub Secrets

1. Go to your GitHub repository
2. Navigate to Settings > Secrets and variables > Actions
3. Click "New repository secret"
4. Name it `RAILWAY_TOKEN`
5. Paste your Railway token as the value

### 3. Configure Your Railway Project

1. Link your GitHub repository to Railway:
   ```bash
   railway link
   ```

2. Set your environment variables:
   ```bash
   railway variables set GEMINI_API_KEY=your_api_key
   ```

### 4. Update railway.toml (Optional)

Make sure your `railway.toml` file is properly configured for your application:

```toml
[build]
builder = "dockerfile"
dockerfilePath = "backend/Dockerfile"

[deploy]
startCommand = "python -m backend.src.api.main"
restartPolicyType = "on-failure"
restartPolicyMaxRetries = 3

[variables]
PYTHON_ENV = "production"
PORT = "8000"
QDRANT_HOST = "localhost"
QDRANT_PORT = "6333"
GEMINI_API_KEY = { from = "GEMINI_API_KEY" }
```

## How It Works

The GitHub Actions workflow in `.github/workflows/railway-deploy.yml` will:

1. Trigger on pushes to main and 001-rag-chatbot branches
2. Install the Railway CLI
3. Authenticate using the stored token
4. Deploy the application to Railway

## Troubleshooting

### Deployment Fails

- Verify your `RAILWAY_TOKEN` secret is correctly set
- Check that your `railway.toml` file is properly configured
- Ensure all required environment variables are set in Railway

### GitHub Actions Not Running

- Make sure the workflow file is in the correct location
- Verify the branch names in the workflow match your repository
- Check that Actions are enabled in your repository settings

## Manual Deployment

If you prefer to deploy manually:

```bash
# Deploy from local machine
railway up

# Or deploy with the CLI
railway deploy
```