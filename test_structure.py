"""
Simple Structure Test for RAG-enabled Humanoid Robotics Chatbot

This script tests the file structure and basic code syntax without requiring external services.
"""

import sys
import os
import importlib.util
from pathlib import Path

def test_file_exists(filepath):
    """Test if a file exists"""
    full_path = Path(filepath)
    exists = full_path.exists()
    status = "PASS" if exists else "FAIL"
    print(f"[{status}] {filepath}")
    return exists

def test_syntax(filepath):
    """Test if a Python file has valid syntax"""
    try:
        with open(filepath, 'r', encoding='utf-8') as file:
            source = file.read()
        compile(source, filepath, 'exec')
        print(f"[PASS] {filepath} - Syntax OK")
        return True
    except SyntaxError as e:
        print(f"[FAIL] {filepath} - Syntax Error: {e}")
        return False
    except Exception as e:
        print(f"[FAIL] {filepath} - Error: {e}")
        return False

def test_structure():
    """Test the complete project structure"""
    print("Testing project structure...")

    required_files = [
        # Backend structure
        "backend/src/api/main.py",
        "backend/src/services/rag.py",
        "backend/src/services/indexer.py",
        "backend/src/services/gemini_client.py",
        "backend/src/services/vector_store.py",
        "backend/src/services/config.py",
        "backend/src/services/session_manager.py",
        "backend/src/models/chunk.py",
        "backend/src/models/query.py",
        "backend/src/models/session.py",
        "backend/src/models/response.py",
        "backend/src/utils/chunking.py",
        "backend/src/utils/markdown_parser.py",

        # Frontend structure
        "frontend/app.py",
        "frontend/src/services/chat_service.py",

        # Configuration
        "backend/requirements.txt",
        "frontend/requirements.txt",
        "backend/.env.example",
        "frontend/.env.example",

        # Docker files
        "backend/Dockerfile",
        "frontend/Dockerfile",

        # Documentation
        "README.md"
    ]

    all_exist = True
    for filepath in required_files:
        full_path = os.path.join(os.getcwd(), filepath)
        if not os.path.exists(full_path):
            print(f"[FAIL] Missing: {filepath}")
            all_exist = False
        else:
            print(f"[PASS] Found: {filepath}")

    return all_exist

def test_syntax_all():
    """Test syntax for all Python files"""
    print("\nTesting Python file syntax...")

    python_files = []

    # Add backend Python files
    for root, dirs, files in os.walk("backend"):
        for file in files:
            if file.endswith(".py"):
                python_files.append(os.path.join(root, file))

    # Add frontend Python files
    for root, dirs, files in os.walk("frontend"):
        for file in files:
            if file.endswith(".py"):
                python_files.append(os.path.join(root, file))

    # Add root Python files
    for file in os.listdir("."):
        if file.endswith(".py") and file != os.path.basename(__file__):
            python_files.append(file)

    all_valid = True
    for filepath in python_files:
        if not test_syntax(filepath):
            all_valid = False

    print(f"\nTested {len(python_files)} Python files")
    return all_valid

def test_basic_imports():
    """Test basic imports without external dependencies"""
    print("\nTesting basic imports...")

    # Temporarily modify sys.path to avoid initialization issues
    original_path = sys.path[:]
    sys.path.insert(0, os.path.join(os.getcwd(), 'backend'))

    try:
        # Test config module (no external dependencies)
        import backend.src.services.config
        print("[PASS] Config module imported")

        # Test model modules (no external dependencies)
        import backend.src.models.chunk
        import backend.src.models.session
        import backend.src.models.query
        import backend.src.models.response
        print("[PASS] Model modules imported")

        # Test utility modules (no external dependencies)
        import backend.src.utils.chunking
        import backend.src.utils.markdown_parser
        print("[PASS] Utility modules imported")

        # Test service modules (avoid those that connect to external services)
        import backend.src.services.session_manager
        print("[PASS] Session manager module imported")

        return True

    except Exception as e:
        print(f"[FAIL] Import error: {e}")
        return False
    finally:
        sys.path = original_path

def main():
    print("Testing Humanoid Robotics Chatbot Structure")
    print("=" * 50)

    # Test 1: Structure
    structure_ok = test_structure()

    # Test 2: Syntax
    syntax_ok = test_syntax_all()

    # Test 3: Basic imports
    imports_ok = test_basic_imports()

    print("\n" + "=" * 50)
    print("STRUCTURE TEST SUMMARY")
    print("=" * 50)
    print(f"Directory Structure: {'PASS' if structure_ok else 'FAIL'}")
    print(f"Syntax Validation: {'PASS' if syntax_ok else 'FAIL'}")
    print(f"Basic Imports: {'PASS' if imports_ok else 'FAIL'}")

    overall_success = structure_ok and syntax_ok and imports_ok
    print(f"Overall Status: {'ALL TESTS PASSED' if overall_success else 'SOME TESTS FAILED'}")

    return overall_success

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)