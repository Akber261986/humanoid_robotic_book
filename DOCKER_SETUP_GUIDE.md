# Setting up Docker and Qdrant for RAG Chatbot

This guide explains how to set up Docker and Qdrant for the RAG chatbot system.

## Prerequisites

- Windows 10 or 11 (with WSL 2 enabled for Docker Desktop)
- At least 4GB of free RAM
- Administrator access to install software

## Installing Docker

### Option 1: Docker Desktop (Recommended)
1. Download Docker Desktop from https://www.docker.com/products/docker-desktop
2. Run the installer as administrator
3. Follow the installation wizard
4. Restart your computer when prompted
5. Launch Docker Desktop and sign in (optional but recommended)

### Option 2: Docker Engine (Alternative)
1. Enable Windows Subsystem for Linux (WSL) 2:
   - Open PowerShell as Administrator
   - Run: `wsl --install`
   - Restart your computer

2. Install Docker Engine following the official guide:
   - Visit: https://docs.docker.com/engine/install/ubuntu/

## Starting Qdrant

Once Docker is installed and running:

1. Open a terminal/command prompt
2. Run the following command to start Qdrant:

```bash
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant
```

3. Verify Qdrant is running:
```bash
curl http://localhost:6333
```

You should receive a JSON response indicating Qdrant is ready.

## Troubleshooting

### Common Issues:

1. **Docker won't start**: Make sure Windows features like Hyper-V and Containers are enabled
2. **Port already in use**: Change the port mapping (e.g., `-p 6334:6333`)
3. **Insufficient memory**: Allocate more memory to Docker Desktop in Settings > Resources

### Checking Qdrant Status:
```bash
docker ps
```
Should show the qdrant-container as running.

### Stopping Qdrant:
```bash
docker stop qdrant-container
```

### Removing Qdrant Container:
```bash
docker rm qdrant-container
```

## Alternative: Qdrant Cloud

If Docker is not suitable for your environment, you can use Qdrant Cloud:
1. Visit https://cloud.qdrant.io/
2. Create an account
3. Create a new cluster
4. Update your `.env` file with the cluster URL and API key

## Next Steps

After Qdrant is running:
1. Add your Gemini API key to `backend/.env`
2. Index your book content by running the indexing script
3. Start the backend server
4. Start the frontend application