#!/usr/bin/env python3
"""
Direct test of rag_simple to see what's happening
"""
import sys
sys.path.append('./backend')

from backend.rag_simple import query_rag_pipeline

print("Testing rag_simple directly with new API key...")
result = query_rag_pipeline("What are the fundamentals of humanoid robotics?")
print("Result:", result)