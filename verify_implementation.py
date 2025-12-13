"""
Quick Verification Script for RAG-enabled Humanoid Robotics Chatbot

This script verifies that all components are properly structured and importable.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_imports():
    """Test that all key modules can be imported without errors"""
    print("Testing module imports...")

    modules_to_test = [
        "backend.src.api.main",
        "backend.src.services.rag",
        "backend.src.services.indexer",
        "backend.src.services.gemini_client",
        "backend.src.services.vector_store",
        "backend.src.services.config",
        "backend.src.models.chunk",
        "backend.src.models.session",
        "backend.src.utils.chunking",
        "backend.src.utils.markdown_parser",
        "frontend.src.services.chat_service"
    ]

    failed_imports = []
    successful_imports = []

    for module_path in modules_to_test:
        try:
            __import__(module_path)
            successful_imports.append(module_path)
            print(f"✓ {module_path}")
        except ImportError as e:
            failed_imports.append((module_path, str(e)))
            print(f"✗ {module_path}: {e}")

    print(f"\nImport Results: {len(successful_imports)} successful, {len(failed_imports)} failed")
    return len(failed_imports) == 0

def test_basic_functionality():
    """Test basic functionality without external dependencies"""
    print("\nTesting basic functionality...")

    try:
        # Test config loading
        from backend.src.services.config import Config
        print(f"✓ Config loaded: Qdrant URL = {Config.QDRANT_URL}")

        # Test models
        from backend.src.models.chunk import BookContentChunk
        from backend.src.models.session import ChatSession
        from datetime import datetime

        chunk = BookContentChunk(
            chunk_id="test_chunk",
            text="This is a test chunk",
            file_path="test.md",
            embedding=[0.1, 0.2, 0.3]
        )
        print(f"✓ BookContentChunk created: {chunk.chunk_id}")

        session = ChatSession(
            session_id="test_session",
            created_at=datetime.now(),
            last_accessed=datetime.now(),
            queries=[]
        )
        print(f"✓ ChatSession created: {session.session_id}")

        return True
    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False

def test_structure():
    """Verify the directory structure is correct"""
    print("\nTesting directory structure...")

    required_paths = [
        "backend/src/api/main.py",
        "backend/src/services/rag.py",
        "backend/src/services/indexer.py",
        "backend/src/models/chunk.py",
        "backend/src/utils/chunking.py",
        "frontend/app.py",
        "frontend/src/services/chat_service.py",
        "backend/requirements.txt",
        "frontend/requirements.txt",
        "backend/Dockerfile",
        "frontend/Dockerfile"
    ]

    missing_paths = []
    for path in required_paths:
        full_path = os.path.join(os.path.dirname(__file__), path)
        if not os.path.exists(full_path):
            missing_paths.append(path)
            print(f"✗ Missing: {path}")
        else:
            print(f"✓ Found: {path}")

    print(f"\nStructure check: {len(required_paths) - len(missing_paths)}/{len(required_paths)} files present")
    return len(missing_paths) == 0

def main():
    print("Verifying Humanoid Robotics Chatbot Implementation")
    print("=" * 55)

    all_tests_passed = True

    # Test 1: Module imports
    imports_ok = test_imports()
    all_tests_passed = all_tests_passed and imports_ok

    # Test 2: Basic functionality
    basic_ok = test_basic_functionality()
    all_tests_passed = all_tests_passed and basic_ok

    # Test 3: Directory structure
    structure_ok = test_structure()
    all_tests_passed = all_tests_passed and structure_ok

    print("\n" + "=" * 55)
    print("VERIFICATION SUMMARY")
    print("=" * 55)
    print(f"Module Imports: {'PASS' if imports_ok else 'FAIL'}")
    print(f"Basic Functionality: {'PASS' if basic_ok else 'FAIL'}")
    print(f"Directory Structure: {'PASS' if structure_ok else 'FAIL'}")
    print(f"Overall Status: {'ALL VERIFICATIONS PASSED' if all_tests_passed else 'SOME VERIFICATIONS FAILED'}")

    return all_tests_passed

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)