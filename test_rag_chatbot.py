"""
Test script to verify the RAG chatbot functionality
"""
import requests
import json
import time

def test_backend_health():
    """Test if the backend is running and accessible"""
    try:
        response = requests.get("http://localhost:8000/health")
        if response.status_code == 200:
            print("✅ Backend health check: PASSED")
            return True
        else:
            print(f"❌ Backend health check: FAILED - Status {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Backend health check: FAILED - {e}")
        return False

def test_service_status():
    """Test if required services (Qdrant, Gemini) are available"""
    try:
        response = requests.get("http://localhost:8000/status")
        if response.status_code == 200:
            status = response.json()
            print(f"✅ Service status: Qdrant={status.get('qdrant')}, Gemini={status.get('gemini_api')}")
            return status.get('qdrant') and status.get('gemini_api')
        else:
            print(f"❌ Service status check: FAILED - Status {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Service status check: FAILED - {e}")
        return False

def test_query_functionality():
    """Test the query functionality with a sample question"""
    sample_queries = [
        "What is humanoid robotics?",
        "Explain inverse kinematics",
        "How do robots maintain balance?"
    ]
    
    all_passed = True
    
    for query in sample_queries:
        try:
            print(f"\nTesting query: '{query}'")
            response = requests.get(f"http://localhost:8000/api/query-book?query={query}")
            
            if response.status_code == 200:
                data = response.json()
                if "response" in data and data["response"]:
                    print(f"✅ Query '{query[:30]}...' : PASSED")
                    print(f"   Response preview: {data['response'][:100]}...")
                    if "sources" in data and data["sources"]:
                        print(f"   Sources found: {len(data['sources'])}")
                else:
                    print(f"❌ Query '{query[:30]}...' : FAILED - No response")
                    all_passed = False
            else:
                print(f"❌ Query '{query[:30]}...' : FAILED - Status {response.status_code}")
                all_passed = False
                
        except Exception as e:
            print(f"❌ Query '{query[:30]}...' : FAILED - {e}")
            all_passed = False
        
        # Add delay to avoid overwhelming the API
        time.sleep(1)
    
    return all_passed

def test_color_scheme():
    """Test if the color scheme has been updated correctly"""
    import os

    # Check if custom.css has the new color
    custom_css_path = "src/css/custom.css"
    if os.path.exists(custom_css_path):
        with open(custom_css_path, 'r', encoding='utf-8') as f:
            content = f.read()
            if "#02c8fa" in content:
                print("✅ Color scheme: Primary color #02c8fa found in custom.css")
                return True
            else:
                print("❌ Color scheme: Primary color #02c8fa NOT found in custom.css")
                return False
    else:
        print("❌ Color scheme: custom.css file not found")
        return False

def main():
    print("🤖 Starting RAG Chatbot End-to-End Test")
    print("="*50)

    # Test 1: Backend health
    health_ok = test_backend_health()

    # Test 2: Color scheme
    color_ok = test_color_scheme()

    if not health_ok:
        print("\n❌ Backend is not accessible. Please start the FastAPI server first.")
        print("Run: cd backend && python -m uvicorn main:app --reload --port 8000")
        return

    # Test 3: Service availability
    services_ok = test_service_status()

    if not services_ok:
        print("\n⚠️  Some services are not available. Check your Qdrant and Gemini API configuration.")

    # Test 4: Query functionality
    queries_ok = test_query_functionality()

    print("\n" + "="*50)
    print("📊 Test Results Summary:")
    print(f"   Backend Health: {'✅ PASSED' if health_ok else '❌ FAILED'}")
    print(f"   Color Scheme: {'✅ PASSED' if color_ok else '❌ FAILED'}")
    print(f"   Service Status: {'✅ PASSED' if services_ok else '⚠️  ISSUE'}")
    print(f"   Query Function: {'✅ PASSED' if queries_ok else '❌ FAILED'}")

    if health_ok and color_ok and queries_ok:
        print("\n🎉 Overall: RAG Chatbot is working correctly with new color scheme!")
    else:
        print("\n❌ Overall: Some issues were found. Please check the above errors.")

if __name__ == "__main__":
    main()