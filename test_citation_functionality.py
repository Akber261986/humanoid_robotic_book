"""
Simple test script to verify citation functionality
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.services.rag import RAGService
from backend.src.services.indexer import Indexer


def test_citation_functionality():
    """
    Test citation display and expansion functionality
    """
    print("Testing citation functionality...")

    # Initialize the RAG service
    rag_service = RAGService()

    # Test a query that should return sources
    test_query = "What is robotics?"

    print(f"\nTesting query: {test_query}")
    result = rag_service.query_rag(test_query)

    print(f"Response: {result['response'][:100]}...")
    print(f"Number of sources: {len(result['sources'])}")

    if result['sources']:
        first_source = result['sources'][0]
        print(f"First source file: {first_source['file_path']}")
        print(f"First source chunk_id: {first_source['chunk_id']}")
        print(f"First source text preview: {first_source['text'][:100]}...")

        # Test retrieving the chunk by ID
        retrieved_chunk = rag_service.get_chunk_by_id(first_source['chunk_id'])
        if retrieved_chunk:
            print(f"Retrieved chunk matches: {retrieved_chunk['text'][:50] == first_source['text'][:50]}")
        else:
            print("Could not retrieve chunk by ID")
    else:
        print("No sources returned - citation functionality cannot be tested without indexed content")

    print("\nCitation functionality test completed.")


def test_index_status():
    """
    Test the index status to see if there's content to query
    """
    print("\nChecking index status...")
    indexer = Indexer()
    status = indexer.check_index_status()
    print(f"Index status: {status}")


if __name__ == "__main__":
    # Test index status
    test_index_status()

    # Test citation functionality
    test_citation_functionality()