#!/usr/bin/env python3
"""
Test script to verify the RAG query functionality
"""
import os
import sys
from dotenv import load_dotenv
import requests
import time

# Load environment variables
load_dotenv()

def test_rag_query():
    """Test the RAG query functionality"""
    print("Testing RAG query functionality...")
    
    # Check if the backend server is running
    try:
        response = requests.get("http://localhost:8000/health", timeout=10)
        if response.status_code == 200:
            print("✅ FastAPI server is running")
        else:
            print("❌ FastAPI server is not responding properly")
            return False
    except requests.exceptions.ConnectionError:
        print("❌ FastAPI server is not running")
        print("   Start it with: cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000")
        return False
    
    # Test service status
    try:
        response = requests.get("http://localhost:8000/status", timeout=10)
        if response.status_code == 200:
            status = response.json()
            print(f"Service status: {status}")
        else:
            print(f"❌ Status check failed with status {response.status_code}")
    except Exception as e:
        print(f"❌ Error checking status: {e}")
    
    # Test a simple query
    test_queries = [
        "What is humanoid robotics?",
        "Explain inverse kinematics",
        "How to set up ROS 2"
    ]
    
    for query in test_queries:
        try:
            print(f"\nTesting query: '{query}'")
            response = requests.get(f"http://localhost:8000/api/query-book?query={query}", timeout=30)
            
            if response.status_code == 200:
                result = response.json()
                print(f"  Response received: {len(result.get('response', ''))} characters")
                print(f"  Retrieved documents: {result.get('retrieved_docs_count', 0)}")
                if 'sources' in result and result['sources']:
                    print(f"  Sources found: {len(result['sources'])}")
                    for i, source in enumerate(result['sources'][:2]):  # Show first 2 sources
                        print(f"    Source {i+1}: {source.get('file', 'Unknown')[:50]}...")
                else:
                    print("  No sources found - this may be expected if no embeddings were successfully created")
            else:
                print(f"  ❌ Query failed with status {response.status_code}: {response.text}")
        except Exception as e:
            print(f"  ❌ Error querying: {e}")
    
    return True

if __name__ == "__main__":
    print("Testing RAG Query Functionality")
    print("="*50)
    
    # Add the project root to Python path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    
    success = test_rag_query()
    
    if success:
        print("\n✅ Query functionality test completed")
        print("\nNote: If no sources are found, it's likely because the embedding process")
        print("hit quota limits during the embedding step. Once you have sufficient quota,")
        print("you can re-run the embedding process and then test queries again.")
    else:
        print("\n❌ Query functionality test failed")