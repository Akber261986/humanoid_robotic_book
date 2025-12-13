"""
Verification script for the RAG Chatbot setup
This script checks if all components are properly structured and can be imported
"""
import sys
import os

def test_backend_structure():
    """Test that backend components can be imported"""
    print("Testing backend structure...")

    # Add backend to path
    backend_path = os.path.join(os.getcwd(), 'backend')
    sys.path.insert(0, backend_path)

    try:
        # Test imports without initializing services that need Qdrant
        from backend.src.services.config import Config
        print("[OK] Config module imported successfully")

        # Validate configuration
        try:
            Config.validate_config()
            print("[OK] Configuration validation passed")
        except ValueError as e:
            print(f"[ERROR] Configuration error: {e}")
            return False

        from backend.src.models.chunk import BookContentChunk
        from backend.src.models.session import ChatSession
        print("[OK] Model modules imported successfully")

        from backend.src.utils.markdown_parser import MarkdownParser
        from backend.src.utils.chunking import create_chunks_from_text
        print("[OK] Utility modules imported successfully")

        # Try to import the main API without initializing services that connect to Qdrant
        # We'll import individual services separately to test their imports
        from backend.src.services.session_manager import SessionManager
        print("[OK] Session manager imported successfully")

        print("[OK] All backend imports successful")
        return True

    except ImportError as e:
        print(f"[ERROR] Backend import error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Backend error: {e}")
        return False

def test_frontend_structure():
    """Test that frontend components can be imported"""
    print("\nTesting frontend structure...")

    try:
        # Add frontend to path
        frontend_path = os.path.join(os.getcwd(), 'frontend')
        sys.path.insert(0, frontend_path)

        from frontend.src.services.chat_service import ChatService
        print("[OK] Frontend chat service imported successfully")

        print("[OK] All frontend imports successful")
        return True

    except ImportError as e:
        print(f"[ERROR] Frontend import error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Frontend error: {e}")
        return False

def check_environment():
    """Check if required environment variables are set"""
    print("\nChecking environment...")

    backend_env_path = os.path.join(os.getcwd(), 'backend', '.env')
    frontend_env_path = os.path.join(os.getcwd(), 'frontend', '.env')

    if os.path.exists(backend_env_path):
        print("[OK] Backend .env file exists")
    else:
        print("[ERROR] Backend .env file missing")

    if os.path.exists(frontend_env_path):
        print("[OK] Frontend .env file exists")
    else:
        print("[ERROR] Frontend .env file missing")

    # Check if required external services are available
    try:
        import google.generativeai
        print("[OK] Google Generative AI library available")
    except ImportError:
        print("[ERROR] Google Generative AI library not available")
        return False

    try:
        import qdrant_client
        print("[OK] Qdrant client library available")
    except ImportError:
        print("[ERROR] Qdrant client library not available")
        return False

    try:
        import fastapi
        print("[OK] FastAPI library available")
    except ImportError:
        print("[ERROR] FastAPI library not available")
        return False

    try:
        import streamlit
        print("[OK] Streamlit library available")
    except ImportError:
        print("[ERROR] Streamlit library not available")
        return False

    return True

def main():
    print("RAG Chatbot Setup Verification")
    print("=" * 40)

    backend_ok = test_backend_structure()
    frontend_ok = test_frontend_structure()
    env_ok = check_environment()

    print("\n" + "=" * 40)
    print("VERIFICATION SUMMARY")
    print("=" * 40)
    print(f"Backend Structure: {'[OK] PASS' if backend_ok else '[ERROR] FAIL'}")
    print(f"Frontend Structure: {'[OK] PASS' if frontend_ok else '[ERROR] FAIL'}")
    print(f"Environment: {'[OK] PASS' if env_ok else '[ERROR] FAIL'}")

    overall_success = backend_ok and frontend_ok and env_ok
    print(f"Overall Status: {'[OK] ALL CHECKS PASSED' if overall_success else '[ERROR] SOME CHECKS FAILED'}")

    if overall_success:
        print("\n[OK] The RAG chatbot system structure is properly set up!")
        print("Next steps:")
        print("1. Install Docker and run Qdrant container")
        print("2. Add your GEMINI_API_KEY to the backend/.env file")
        print("3. Index your book content")
        print("4. Start the backend and frontend services")
    else:
        print("\n[ERROR] There are issues with the setup that need to be resolved.")

    return overall_success

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)