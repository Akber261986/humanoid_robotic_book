#!/usr/bin/env python3
"""
Test script to verify the RAG functionality once the book is embedded
"""

import requests
import time
import sys
import os

def check_services():
    """Check if required services are running"""
    print("Checking if services are running...")
    
    # Check if FastAPI server is running
    try:
        response = requests.get("http://localhost:8000/health", timeout=5)
        if response.status_code == 200:
            print("✅ FastAPI server is running")
        else:
            print("❌ FastAPI server is not responding properly")
            return False
    except requests.exceptions.ConnectionError:
        print("❌ FastAPI server is not running (connection error)")
        print("   Start it with: cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000")
        return False
    except Exception as e:
        print(f"❌ Error checking FastAPI server: {e}")
        return False
    
    # Check if Qdrant is running
    try:
        response = requests.get("http://localhost:8000/status", timeout=10)
        if response.status_code == 200:
            status = response.json()
            if status.get("qdrant"):
                print("✅ Qdrant connection is working")
            else:
                print("❌ Qdrant is not connected properly")
                print(f"   Status: {status}")
                return False
        else:
            print("❌ Could not get status from FastAPI server")
            return False
    except Exception as e:
        print(f"❌ Error checking Qdrant connection: {e}")
        return False
    
    return True

def test_query():
    """Test querying the embedded book content"""
    print("\nTesting query functionality...")
    
    test_queries = [
        "What is humanoid robotics?",
        "Explain inverse kinematics in robotics",
        "How to set up ROS 2 for robotics",
        "What is Isaac Sim?",
        "Vision-Language Assistant in robotics"
    ]
    
    for query in test_queries:
        try:
            print(f"\nTesting query: '{query}'")
            response = requests.get(f"http://localhost:8000/api/query-book?query={query}", timeout=30)
            
            if response.status_code == 200:
                result = response.json()
                print(f"  Response length: {len(result.get('response', ''))} characters")
                print(f"  Retrieved documents: {result.get('retrieved_docs_count', 0)}")
                
                if result.get('sources'):
                    print(f"  Sources: {len(result['sources'])}")
                    for i, source in enumerate(result['sources'][:2]):  # Show first 2 sources
                        print(f"    Source {i+1}: {source.get('file', 'Unknown file')[:50]}...")
                else:
                    print("  No sources found - book content may not be embedded yet")
            else:
                print(f"  ❌ Query failed with status {response.status_code}: {response.text}")
        except Exception as e:
            print(f"  ❌ Error querying: {e}")
    
    return True

def main():
    print("Humanoid Robotics Book - Query Test")
    print("="*50)
    
    # Check if services are running
    if not check_services():
        print("\n❌ Required services are not running.")
        print("Please ensure:")
        print("1. Docker is installed and running")
        print("2. Qdrant container is started: docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant")
        print("3. FastAPI server is running: cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000")
        print("4. Book content is embedded: curl -X POST http://localhost:8000/embed-book")
        return False
    
    print("\n✅ All services are running correctly")
    
    # Test querying
    test_query()
    
    print("\n" + "="*50)
    print("Test completed. If queries return sources, the book content has been successfully embedded.")
    print("If no sources are returned, you may need to embed the book content first.")
    
    return True

if __name__ == "__main__":
    main()