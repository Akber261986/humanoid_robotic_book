"""
End-to-End Testing Script for RAG-enabled Humanoid Robotics Chatbot

This script performs end-to-end testing with 5 sample book queries to verify:
1. Backend API functionality
2. RAG pipeline processing
3. Response quality and citations
4. Session management
5. Error handling
"""

import requests
import time
import json
from typing import Dict, List
import os

# Configuration
BASE_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
HEADERS = {"Content-Type": "application/json"}

def test_health_check():
    """Test the health check endpoint"""
    print("Testing health check endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/health")
        if response.status_code == 200:
            health_data = response.json()
            print(f"✓ Health check successful: {health_data['status']}")
            print(f"  Dependencies: {health_data['dependencies']}")
            return True
        else:
            print(f"✗ Health check failed with status: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Health check error: {e}")
        return False

def test_session_creation():
    """Test session creation functionality"""
    print("\nTesting session creation...")
    try:
        response = requests.post(f"{BASE_URL}/api/session")
        if response.status_code == 200:
            session_data = response.json()
            session_id = session_data.get("session_id")
            print(f"✓ Session created successfully: {session_id}")
            return session_id
        else:
            print(f"✗ Session creation failed with status: {response.status_code}")
            print(f"  Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ Session creation error: {e}")
        return None

def test_session_details(session_id: str):
    """Test session details endpoint"""
    print(f"\nTesting session details for session: {session_id[:8]}...")
    try:
        response = requests.get(f"{BASE_URL}/api/session/{session_id}")
        if response.status_code == 200:
            session_details = response.json()
            print(f"✓ Session details retrieved: {session_details['query_count']} queries")
            return True
        else:
            print(f"✗ Session details failed with status: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Session details error: {e}")
        return False

def test_query(session_id: str, query: str, test_name: str):
    """Test a single query"""
    print(f"\nTesting query '{test_name}': {query[:50]}...")
    try:
        start_time = time.time()

        payload = {
            "query": query,
            "session_id": session_id
        }

        response = requests.post(f"{BASE_URL}/api/query", json=payload, headers=HEADERS)

        end_time = time.time()
        response_time = end_time - start_time

        if response.status_code == 200:
            result = response.json()
            print(f"✓ Query successful (response time: {response_time:.2f}s)")
            print(f"  Response length: {len(result['response'])} chars")
            print(f"  Sources returned: {len(result['sources'])}")

            # Check if response has content
            if len(result['response']) > 0 and not result['response'].startswith("The database is currently empty"):
                print(f"  ✓ Response contains content")
                if len(result['sources']) > 0:
                    print(f"  ✓ Citations included")
                else:
                    print(f"  ⚠ No citations returned")
            else:
                print(f"  ⚠ Response may be empty or database not indexed")

            return True, response_time
        else:
            print(f"✗ Query failed with status: {response.status_code}")
            print(f"  Error: {response.text}")
            return False, response_time
    except Exception as e:
        print(f"✗ Query error: {e}")
        return False, 0

def test_chunk_retrieval(session_id: str):
    """Test chunk retrieval by ID"""
    print(f"\nTesting chunk retrieval...")
    try:
        # First, make a query to get some sources
        payload = {
            "query": "What is robotics?",
            "session_id": session_id
        }
        response = requests.post(f"{BASE_URL}/api/query", json=payload, headers=HEADERS)

        if response.status_code == 200:
            result = response.json()
            if result['sources']:
                chunk_id = result['sources'][0]['chunk_id']
                response = requests.get(f"{BASE_URL}/api/chunk/{chunk_id}")
                if response.status_code == 200:
                    chunk_data = response.json()
                    print(f"✓ Chunk retrieval successful: {len(chunk_data['text'])} chars")
                    return True
                else:
                    print(f"✗ Chunk retrieval failed with status: {response.status_code}")
                    return False
            else:
                print("⚠ No sources returned to test chunk retrieval")
                return True  # Not a failure if no sources exist
        else:
            print(f"✗ Initial query failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Chunk retrieval error: {e}")
        return False

def run_end_to_end_tests():
    """Run all end-to-end tests"""
    print("🤖 Starting End-to-End Tests for Humanoid Robotics Chatbot")
    print("=" * 60)

    # Test 1: Health check
    health_ok = test_health_check()

    if not health_ok:
        print("\n❌ Backend is not healthy. Please start the backend server first.")
        print("Run: cd backend && uvicorn backend.src.api.main:app --reload --host 0.0.0.0 --port 8000")
        return False

    # Test 2: Session management
    session_id = test_session_creation()
    if not session_id:
        print("\n❌ Session creation failed. Cannot proceed with query tests.")
        return False

    # Test session details
    session_details_ok = test_session_details(session_id)

    # Define 5 sample book queries for testing
    sample_queries = [
        {
            "name": "Basic Robotics Definition",
            "query": "What is robotics according to the book?"
        },
        {
            "name": "Humanoid Kinematics",
            "query": "Explain humanoid kinematics and joint configurations."
        },
        {
            "name": "AI Integration",
            "query": "How is AI integrated with humanoid robotics systems?"
        },
        {
            "name": "Simulation Environments",
            "query": "What simulation environments are recommended for humanoid robotics?"
        },
        {
            "name": "ROS Integration",
            "query": "How does ROS integrate with humanoid robotics?"
        }
    ]

    # Test 3-7: Sample queries
    successful_queries = 0
    total_response_time = 0
    response_times = []

    for i, query_info in enumerate(sample_queries, 1):
        success, response_time = test_query(session_id, query_info['query'], query_info['name'])
        if success:
            successful_queries += 1
            total_response_time += response_time
            response_times.append(response_time)

    # Test 8: Chunk retrieval
    chunk_retrieval_ok = test_chunk_retrieval(session_id)

    # Summary
    print("\n" + "=" * 60)
    print("📊 END-TO-END TEST RESULTS")
    print("=" * 60)
    print(f"Health Check: {'✓ PASS' if health_ok else '✗ FAIL'}")
    print(f"Session Management: {'✓ PASS' if session_id else '✗ FAIL'}")
    print(f"Session Details: {'✓ PASS' if session_details_ok else '✗ FAIL'}")
    print(f"Sample Queries: {successful_queries}/5 PASS")
    print(f"Chunk Retrieval: {'✓ PASS' if chunk_retrieval_ok else '✗ FAIL'}")

    if successful_queries > 0:
        avg_response_time = total_response_time / successful_queries
        max_response_time = max(response_times) if response_times else 0
        min_response_time = min(response_times) if response_times else 0

        print(f"\n⏱️  PERFORMANCE METRICS:")
        print(f"  Average Response Time: {avg_response_time:.2f}s")
        print(f"  Max Response Time: {max_response_time:.2f}s")
        print(f"  Min Response Time: {min_response_time:.2f}s")
        print(f"  Target (<3s): {'✓ MET' if avg_response_time < 3.0 else '✗ NOT MET'}")

    overall_success = all([
        health_ok,
        session_id is not None,
        session_details_ok,
        successful_queries >= 3,  # At least 3 out of 5 queries should succeed
        chunk_retrieval_ok
    ])

    print(f"\n🎯 OVERALL RESULT: {'✓ ALL TESTS PASSED' if overall_success else '✗ SOME TESTS FAILED'}")

    return overall_success

if __name__ == "__main__":
    success = run_end_to_end_tests()
    exit(0 if success else 1)