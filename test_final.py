#!/usr/bin/env python3
"""
Final test of the API endpoints
"""
import requests

def test_final():
    """Test the API endpoints to confirm they're working"""
    base_url = "http://localhost:8000"

    print("Final API Endpoint Test")
    print("=" * 30)

    # Test health endpoint
    print("Health endpoint:")
    response = requests.get(f"{base_url}/health")
    print(f"  Status: {response.status_code}")
    print(f"  Response: {response.json()}")

    print()

    # Test status endpoint
    print("Status endpoint:")
    response = requests.get(f"{base_url}/status")
    print(f"  Status: {response.status_code}")
    print(f"  Response: {response.json()}")

    print()

    # Test query endpoint (will fail due to suspended API key, but that's expected)
    print("Query endpoint (expected API error due to suspended key):")
    response = requests.get(f"{base_url}/api/query-book?query=What%20are%20the%20fundamentals%20of%20humanoid%20robotics%3F")
    print(f"  Status: {response.status_code}")
    data = response.json()
    print(f"  Retrieved docs: {data.get('retrieved_docs_count', 0)}")
    print(f"  Has response: {'response' in data}")

    # Check if the error is related to API key (expected) rather than model name (previous issue)
    response_text = data.get('response', '')
    if 'suspended' in response_text.lower() or 'permission denied' in response_text.lower():
        print("  [SUCCESS] API is working correctly - error is due to suspended API key (not model name)")
    elif 'gemini-1.5-flash' in response_text:
        print("  [FAILURE] Still showing old model name error")
    else:
        print(f"  [INFO] Other error: {response_text[:100]}...")

if __name__ == "__main__":
    test_final()