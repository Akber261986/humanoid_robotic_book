"""Wrapper to handle warnings before importing the main app."""

import warnings
# Filter out the specific fastembed deprecation warning
warnings.filterwarnings("ignore", message=".*DefaultEmbedding, FlagEmbedding, JinaEmbedding are deprecated.*", category=UserWarning, module="fastembed.embedding")

# Import and run the main application
from api.main import app

if __name__ == "__main__":
    import uvicorn
    import os
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)