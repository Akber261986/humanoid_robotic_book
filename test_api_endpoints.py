#!/usr/bin/env python3
"""
Direct API test to verify the RAG system endpoints work properly
"""
import requests
import time

def test_api_endpoints():
    """Test all API endpoints"""
    base_url = "http://localhost:8000"

    print("Testing Humanoid Robotics Book API Endpoints")
    print("=" * 50)

    # Test 1: Health endpoint
    print("1. Testing /health endpoint...")
    try:
        response = requests.get(f"{base_url}/health")
        print(f"   Status: {response.status_code}")
        print(f"   Response: {response.json()}")
        if response.status_code == 200:
            print("   [OK] Health endpoint working")
        else:
            print("   [ERROR] Health endpoint failed")
    except Exception as e:
        print(f"   ✗ Health endpoint error: {e}")

    print()

    # Test 2: Status endpoint
    print("2. Testing /status endpoint...")
    try:
        response = requests.get(f"{base_url}/status")
        print(f"   Status: {response.status_code}")
        status_data = response.json()
        print(f"   Response: {status_data}")
        if response.status_code == 200:
            print("   [OK] Status endpoint working")
            if status_data.get('qdrant', False):
                print("   [OK] Qdrant connection available")
            if status_data.get('tfidf_vectorizer', False):
                print("   [OK] TF-IDF vectorizer available")
        else:
            print("   [ERROR] Status endpoint failed")
    except Exception as e:
        print(f"   [ERROR] Status endpoint error: {e}")

    print()

    # Test 3: Query endpoint with various questions
    print("3. Testing /api/query-book endpoint...")
    test_questions = [
        "What are the fundamentals of humanoid robotics?",
        "How does Gazebo work for robotics simulation?",
        "Explain Isaac Sim for robotics simulation",
        "What is ROS2 and how is it used in robotics?"
    ]

    for i, question in enumerate(test_questions, 1):
        print(f"   Question {i}: {question}")
        try:
            # URL encode the question
            import urllib.parse
            encoded_question = urllib.parse.quote(question)
            response = requests.get(f"{base_url}/api/query-book?query={encoded_question}")

            print(f"   Status: {response.status_code}")
            if response.status_code == 200:
                data = response.json()
                print(f"   Retrieved docs: {data.get('retrieved_docs_count', 0)}")
                if data.get('sources'):
                    print(f"   Sources: {len(data['sources'])}")
                    for j, source in enumerate(data['sources'][:1], 1):  # Show first source
                        print(f"     {j}. From: {source['file'][:50]}...")
                print(f"   Response preview: {data.get('response', '')[:100]}...")
                print("   [OK] Query successful")
            else:
                print(f"   Response: {response.text}")
                print("   [ERROR] Query failed")
        except Exception as e:
            print(f"   [ERROR] Query error: {e}")

        print()
        time.sleep(1)  # Small delay between requests

if __name__ == "__main__":
    test_api_endpoints()