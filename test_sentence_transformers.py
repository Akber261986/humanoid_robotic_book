#!/usr/bin/env python3
"""
Simple test to check if sentence-transformers works without PyTorch issues
"""
def test_sentence_transformers():
    try:
        from sentence_transformers import SentenceTransformer
        print("✅ Sentence transformers imported successfully")
        
        # Try to create a model instance
        model = SentenceTransformer('all-MiniLM-L6-v2')
        print("✅ Model loaded successfully")
        
        # Try to encode a simple sentence
        sentences = ["This is a test sentence."]
        embeddings = model.encode(sentences)
        print(f"✅ Encoding successful, embedding shape: {embeddings.shape}")
        print(f"✅ Embedding length: {len(embeddings[0])}")
        
        return True
    except Exception as e:
        print(f"X Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Testing sentence-transformers compatibility...")
    success = test_sentence_transformers()

    if success:
        print("\nV Sentence transformers is working properly!")
    else:
        print("\nX Sentence transformers has compatibility issues.")
        print("You may need to install CPU-only PyTorch or use a different approach.")