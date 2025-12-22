#!/usr/bin/env python3
"""
Simple chatbot that reads book content directly and uses Gemini API
This bypasses all TF-IDF and Qdrant complexity
"""
import os
import glob
import markdown
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
from dotenv import load_dotenv
import re

# Load environment
load_dotenv()

# Configure Gemini
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_GENERATION_MODEL = os.getenv("GEMINI_GENERATION_MODEL", "gemini-2.5-flash")

if GEMINI_API_KEY:
    genai.configure(api_key=GEMINI_API_KEY)
    print("Gemini API configured successfully")
else:
    print("ERROR: GEMINI_API_KEY not found in environment")

app = FastAPI(title="Simple Humanoid Robotics Book Chatbot", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def load_book_content():
    """Load all book content from docs directory"""
    docs_path = "./docs"
    all_content = []

    markdown_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    print(f"Found {len(markdown_files)} markdown files")

    for file_path in markdown_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Convert markdown to plain text
            html = markdown.markdown(content)
            text = html.replace('<p>', ' ').replace('</p>', ' ').replace('<h1>', ' ').replace('</h1>', ' ') \
                      .replace('<h2>', ' ').replace('</h2>', ' ').replace('<h3>', ' ').replace('</h3>', ' ') \
                      .replace('<li>', ' ').replace('</li>', ' ').replace('<ul>', ' ').replace('</ul>', ' ') \
                      .replace('<strong>', ' ').replace('</strong>', ' ').replace('<em>', ' ').replace('</em>', ' ')

            text = re.sub(r'\s+', ' ', text).strip()

            all_content.append({
                "source": file_path,
                "content": text[:2000]  # Limit content size
            })

            print(f"Loaded content from {file_path} ({len(text)} chars)")
        except Exception as e:
            print(f"Error loading {file_path}: {e}")

    return all_content

# Load book content at startup
book_content = load_book_content()
print(f"Loaded {len(book_content)} book sections")

class QueryRequest(BaseModel):
    query: str

@app.get("/")
def health():
    return {"status": "healthy", "service": "Simple Humanoid Robotics Book Chatbot"}

@app.get("/api/query-book")
def query_book(query: str = None):
    """Simple query endpoint that uses Gemini with book context"""
    if not query:
        return {"error": "Please provide a query parameter"}

    if not GEMINI_API_KEY:
        return {"error": "GEMINI_API_KEY not configured"}

    try:
        # Combine all book content as context
        context = "\n\n".join([section["content"] for section in book_content])

        # Limit context size to avoid token limits
        if len(context) > 30000:  # Reduce context size
            context = context[:30000]

        prompt = f"""
        You are an AI assistant for the Humanoid Robotics Book.
        Use the following book content to answer the user's question:

        BOOK CONTENT:
        {context}

        USER QUESTION: {query}

        Please provide an answer based on the book content. If the content doesn't contain relevant information,
        provide general information about humanoid robotics that relates to the question.
        """

        model = genai.GenerativeModel(GEMINI_GENERATION_MODEL)
        response = model.generate_content(
            prompt,
            generation_config={
                "temperature": 0.7,
                "max_output_tokens": 1000,
            }
        )

        if response.text:
            return {
                "response": response.text,
                "sources": [{"file": section["source"], "preview": section["content"][:100] + "..."} for section in book_content[:3]],  # Show first 3 sources
                "query": query,
                "retrieved_docs_count": len([s for s in book_content if query.lower() in s["content"].lower()])  # Count docs containing query
            }
        else:
            return {
                "response": "I couldn't generate a response. Please try rephrasing your question.",
                "sources": [],
                "query": query,
                "retrieved_docs_count": 0
            }

    except Exception as e:
        return {
            "response": f"Error generating response: {str(e)}",
            "sources": [],
            "query": query,
            "retrieved_docs_count": 0
        }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)