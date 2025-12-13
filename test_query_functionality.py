"""
Simple test script to verify basic query functionality
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.services.rag import RAGService
from backend.src.services.indexer import Indexer
from backend.src.services.config import Config


def test_basic_query():
    """
    Test basic query functionality with sample questions
    """
    print("Testing basic query functionality...")

    # Initialize the RAG service
    rag_service = RAGService()

    # Test queries related to humanoid robotics
    test_queries = [
        "What is inverse kinematics?",
        "Explain humanoid robotics",
        "How do robots maintain balance?",
        "What are actuators in robotics?",
        "Explain forward kinematics"
    ]

    print("\nRunning sample queries...")
    for i, query in enumerate(test_queries, 1):
        print(f"\n{i}. Query: {query}")
        try:
            result = rag_service.query_rag(query)
            print(f"   Response: {result['response'][:200]}..." if len(result['response']) > 200 else f"   Response: {result['response']}")
            print(f"   Sources: {len(result['sources'])} found")
        except Exception as e:
            print(f"   Error: {e}")

    print("\nQuery functionality test completed.")


def test_index_status():
    """
    Test the index status to see if there's content to query
    """
    print("\nChecking index status...")
    indexer = Indexer()
    status = indexer.check_index_status()
    print(f"Index status: {status}")


if __name__ == "__main__":
    # Validate configuration first
    try:
        Config.validate_config()
        print("Configuration validation passed.")
    except ValueError as e:
        print(f"Configuration error: {e}")
        sys.exit(1)

    # Test index status
    test_index_status()

    # Test basic query functionality
    test_basic_query()