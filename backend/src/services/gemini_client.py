import google.generativeai as genai
from .config import Config
import logging
from typing import List, Optional
import time
import random
from functools import wraps

logger = logging.getLogger(__name__)

# Configure the Gemini API
genai.configure(api_key=Config.GEMINI_API_KEY)


def retry_with_exponential_backoff(max_retries: int = 3, base_delay: float = 1.0, max_delay: float = 60.0):
    """
    Decorator to implement retry logic with exponential backoff
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            last_exception = None
            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt == max_retries:
                        # Last attempt, raise the exception
                        logger.error(f"Failed after {max_retries} retries: {e}")
                        raise e

                    # Calculate delay with exponential backoff and jitter
                    delay = min(base_delay * (2 ** attempt), max_delay)
                    jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                    actual_delay = delay + jitter

                    logger.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {actual_delay:.2f}s...")
                    time.sleep(actual_delay)

            # This line should never be reached, but included for type safety
            raise last_exception
        return wrapper
    return decorator


class GeminiClient:
    """
    Service class to handle interactions with Google's Gemini API
    """

    def __init__(self):
        self.generation_model = genai.GenerativeModel(Config.GEMINI_GENERATION_MODEL)

    @retry_with_exponential_backoff(max_retries=3, base_delay=1.0, max_delay=60.0)
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the given text using Gemini
        """
        response = genai.embed_content(
            model=Config.GEMINI_EMBEDDING_MODEL,
            content=text,
            task_type="RETRIEVAL_DOCUMENT"  # or "RETRIEVAL_QUERY" for queries
        )
        return response['embedding']

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a batch
        """
        embeddings = []
        for text in texts:
            try:
                embedding = self.generate_embedding(text)
                embeddings.append(embedding)
            except Exception as e:
                logger.error(f"Error generating embedding for text: {e}")
                # Add a zero vector as fallback
                embeddings.append([0.0] * 768)  # 768 is the dimension for embedding-001
        return embeddings

    @retry_with_exponential_backoff(max_retries=3, base_delay=1.0, max_delay=60.0)
    def generate_response(self, prompt: str) -> str:
        """
        Generate a response based on the given prompt using Gemini
        """
        response = self.generation_model.generate_content(
            prompt,
            generation_config={
                "max_output_tokens": 1000,  # Approximate to 300 words
                "temperature": 0.3,  # Lower temperature for more factual responses
            }
        )
        return response.text if response.text else ""

    def generate_response_with_citations(self, query: str, retrieved_chunks: List[dict]) -> str:
        """
        Generate a response based on the query and retrieved chunks, with proper citations
        """
        if not retrieved_chunks:
            return "I couldn't find any relevant information in the book to answer your question."

        # Build context from retrieved chunks
        context_parts = []
        for chunk in retrieved_chunks:
            context_parts.append(f"Source: {chunk['file_path']}\nContent: {chunk['text']}\n")

        context = "\n".join(context_parts)

        # Create the prompt with context
        prompt = f"""
        Based on the following book excerpts, please answer the user's question.
        Make sure to cite the sources and keep the response educational and concise (under {Config.MAX_RESPONSE_WORDS} words).

        Book Excerpts:
        {context}

        User Question: {query}

        Please provide a response based on the book content with citations to the relevant sections.
        """

        return self.generate_response(prompt)

    @retry_with_exponential_backoff(max_retries=2, base_delay=1.0, max_delay=30.0)
    def test_connection(self) -> bool:
        """
        Test the connection to the Gemini API
        """
        # Test with a simple embedding
        test_embedding = self.generate_embedding("test")
        return len(test_embedding) > 0