import requests
from typing import Dict, Any, Optional, List


class ChatService:
    """
    Service class to handle communication with the backend API
    """

    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url

    def create_session(self) -> Dict[str, Any]:
        """
        Create a new chat session
        """
        try:
            response = requests.post(f"{self.base_url}/api/session")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error creating session: {e}")
            return {"error": str(e)}

    def query(self, query: str, session_id: str) -> Dict[str, Any]:
        """
        Send a query to the backend and get a response
        """
        try:
            payload = {
                "query": query,
                "session_id": session_id
            }
            response = requests.post(f"{self.base_url}/api/query", json=payload)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error querying: {e}")
            return {"error": str(e)}

    def get_session_details(self, session_id: str) -> Dict[str, Any]:
        """
        Get details about a specific session
        """
        try:
            response = requests.get(f"{self.base_url}/api/session/{session_id}")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error getting session details: {e}")
            return {"error": str(e)}

    def health_check(self) -> Dict[str, Any]:
        """
        Check the health of the backend service
        """
        try:
            response = requests.get(f"{self.base_url}/health")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error checking health: {e}")
            return {"error": str(e)}

    def index_documents(self, docs_path: str) -> Dict[str, Any]:
        """
        Index documents from a directory
        """
        try:
            payload = {"docs_path": docs_path}
            response = requests.post(f"{self.base_url}/api/index", json=payload)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            print(f"Error indexing documents: {e}")
            return {"error": str(e)}