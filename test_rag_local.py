#!/usr/bin/env python3
"""
Test the rag_local module directly
"""
import os
import sys
sys.path.append('./backend')

from backend.rag_local import query_rag_pipeline

def test_rag_local():
    print("Testing rag_local module directly...")

    # Test a simple query
    result = query_rag_pipeline("What are the fundamentals of humanoid robotics?")

    print("Result:", result)
    print("Response preview:", result.get('response', '')[:200] + "..." if result.get('response') else "No response")
    print("Retrieved docs:", result.get('retrieved_docs_count', 0))

if __name__ == "__main__":
    test_rag_local()