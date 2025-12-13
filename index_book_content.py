"""
Script to index the book content into the vector database
This script will process all Markdown files in the docs directory and store them in Qdrant
"""
import os
import sys
import time

def index_book_content():
    """Index all book content from the docs directory"""
    print("Starting book content indexing...")

    # Add backend to path
    backend_path = os.path.join(os.getcwd(), 'backend')
    sys.path.insert(0, backend_path)

    try:
        # Import required modules
        from backend.src.services.indexer import Indexer
        from backend.src.services.config import Config

        # Validate configuration first
        try:
            Config.validate_config()
            print("[OK] Configuration validated")
        except ValueError as e:
            print(f"[ERROR] Configuration error: {e}")
            print("Please ensure GEMINI_API_KEY is set in backend/.env")
            return False

        # Check if Qdrant is available
        print("Checking Qdrant connection...")
        indexer = Indexer()

        # Check if Qdrant is running
        try:
            status = indexer.check_index_status()
            print(f"[OK] Qdrant connection successful")
            print(f"Current index status: {status}")
        except Exception as e:
            print(f"[ERROR] Cannot connect to Qdrant: {e}")
            print("Make sure Qdrant is running before proceeding")
            return False

        # Get the docs directory path
        docs_path = os.path.join(os.getcwd(), 'docs')
        if not os.path.exists(docs_path):
            print(f"[ERROR] Docs directory not found: {docs_path}")
            return False

        print(f"Indexing content from: {docs_path}")

        # Perform indexing
        start_time = time.time()
        result = indexer.index_directory(docs_path)
        end_time = time.time()

        print(f"[OK] Indexing completed in {end_time - start_time:.2f} seconds")
        print(f"Results: {result}")

        return True

    except ImportError as e:
        print(f"[ERROR] Import error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Indexing failed: {e}")
        return False

def main():
    print("Book Content Indexing Script")
    print("=" * 40)

    success = index_book_content()

    print("\n" + "=" * 40)
    if success:
        print("[OK] Book content indexing completed successfully!")
        print("The RAG system is now ready to answer questions about the book content.")
    else:
        print("[ERROR] Book content indexing failed!")
        print("Please check the error messages above and try again.")

    return success

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)