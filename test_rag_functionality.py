#!/usr/bin/env python3
"""
Test script to verify the RAG system works with the embedded book content
"""
import sys
import os
sys.path.append('./backend')

from backend.rag import query_rag_pipeline, check_services

def test_rag_system():
    """Test the RAG system with sample queries about the book content"""
    print("Testing RAG System with Embedded Book Content")
    print("=" * 50)

    # Check if services are available
    print("Checking services...")
    services_status = check_services()
    print(f"Qdrant available: {services_status['qdrant']}")
    print(f"Local embeddings available: {services_status['local_embeddings']}")
    print(f"Collection: {services_status['collection']}")
    print()

    # Test queries related to humanoid robotics
    test_queries = [
        "What are the fundamentals of humanoid robotics?",
        "Explain inverse kinematics in humanoid robots",
        "What is ROS2 and how is it used in robotics?",
        "How does Gazebo work for robotics simulation?",
        "What are the key concepts in physical AI?",
        "Explain Isaac Sim for robotics simulation",
        "What are VLA models in robotics?",
        "How to set up a humanoid robot simulation environment?"
    ]

    print("Running test queries...")
    print()

    for i, query in enumerate(test_queries, 1):
        print(f"Query {i}: {query}")
        print("-" * 30)

        try:
            result = query_rag_pipeline(query)

            print(f"Response: {result['response'][:500]}...")  # First 500 chars
            print(f"Retrieved documents: {result['retrieved_docs_count']}")

            if result['sources']:
                print("Sources:")
                for j, source in enumerate(result['sources'][:2], 1):  # Show first 2 sources
                    print(f"  {j}. File: {source['file']}")
                    print(f"     Score: {source['score']}")
                    print(f"     Preview: {source['text'][:100]}...")
            else:
                print("No sources retrieved")

        except Exception as e:
            print(f"Error processing query: {e}")

        print("\n" + "="*50 + "\n")

if __name__ == "__main__":
    test_rag_system()