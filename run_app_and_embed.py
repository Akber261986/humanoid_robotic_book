#!/usr/bin/env python3
"""
Script to run the FastAPI app and trigger the book embedding process
"""

import subprocess
import time
import requests
import threading
import sys
import os

def start_fastapi_server():
    """Start the FastAPI server in a separate thread"""
    import uvicorn
    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/backend")
    from backend.main import app
    
    uvicorn.run(app, host="0.0.0.0", port=8000)

def check_server_health():
    """Check if the FastAPI server is running"""
    try:
        response = requests.get("http://localhost:8000/health", timeout=5)
        return response.status_code == 200
    except:
        return False

def check_qdrant_connection():
    """Check if Qdrant is accessible"""
    try:
        response = requests.get("http://localhost:6333", timeout=5)
        return response.status_code == 200
    except:
        return False

def trigger_embedding():
    """Trigger the book embedding process via the API"""
    try:
        print("Triggering book embedding process...")
        response = requests.post("http://localhost:8000/embed-book", timeout=300)  # 5 minute timeout
        
        if response.status_code == 200:
            result = response.json()
            print(f"Embedding result: {result}")
            return result.get('status') == 'success'
        else:
            print(f"Failed to trigger embedding. Status code: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"Error triggering embedding: {e}")
        return False

def main():
    print("Checking prerequisites...")
    
    # Check if Qdrant is running
    if not check_qdrant_connection():
        print("❌ Qdrant is not running!")
        print("Please start Qdrant first using:")
        print("docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant")
        print("\nThen run this script again.")
        return False
    
    print("✅ Qdrant is accessible")
    
    # Check if Gemini API key is set
    gemini_api_key = os.getenv("GEMINI_API_KEY") or "your_google_gemini_api_key_here"
    if gemini_api_key == "your_google_gemini_api_key_here":
        print("⚠️  Warning: GEMINI_API_KEY is not set in environment variables.")
        print("Please update your .env file with a valid Gemini API key.")
        print("Get your API key from: https://aistudio.google.com/")
        return False
    
    print("✅ Environment variables are set")
    
    # Start the FastAPI server in a background process
    print("Starting FastAPI server...")
    server_process = subprocess.Popen([sys.executable, "-c", """
import sys
import os
sys.path.insert(0, os.path.join(os.getcwd(), 'backend'))
import uvicorn
from backend.main import app
uvicorn.run(app, host='0.0.0.0', port=8000)
"""], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Give the server some time to start
    print("Waiting for server to start...")
    time.sleep(5)
    
    # Check if server started successfully
    if not check_server_health():
        print("❌ Failed to start FastAPI server")
        server_process.terminate()
        return False
    
    print("✅ FastAPI server is running")
    
    try:
        # Trigger the embedding process
        success = trigger_embedding()
        
        if success:
            print("🎉 Book embedding completed successfully!")
        else:
            print("❌ Book embedding failed!")
            
    finally:
        # Stop the server
        print("Stopping FastAPI server...")
        server_process.terminate()
        server_process.wait()
    
    return success

if __name__ == "__main__":
    print("Humanoid Robotics Book - Embedding Script")
    print("="*50)
    
    success = main()
    
    if success:
        print("\n✅ Process completed successfully!")
        print("You can now query the book content using the API.")
    else:
        print("\n❌ Process failed. Please check the error messages above.")
        print("\nTroubleshooting tips:")
        print("1. Make sure Docker is installed and running")
        print("2. Start Qdrant with: docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant")
        print("3. Set your GEMINI_API_KEY in the .env file")
        print("4. Verify all prerequisites are met")