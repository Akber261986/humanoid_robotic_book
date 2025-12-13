from typing import Dict, Optional, List
from datetime import datetime
import uuid
import threading
from ..models.session import ChatSession


class SessionManager:
    """
    Service class to manage chat sessions in memory
    In production, this would use Redis or a database
    """

    def __init__(self):
        self._sessions: Dict[str, ChatSession] = {}
        self._lock = threading.Lock()  # For thread safety

    def create_session(self) -> ChatSession:
        """
        Create a new chat session
        """
        with self._lock:
            session_id = str(uuid.uuid4())
            session = ChatSession(
                session_id=session_id,
                created_at=datetime.now(),
                last_accessed=datetime.now(),
                queries=[]
            )
            self._sessions[session_id] = session
            return session

    def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Get a session by ID
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                # Update last accessed time
                session.last_accessed = datetime.now()
            return session

    def update_session(self, session: ChatSession):
        """
        Update an existing session
        """
        with self._lock:
            self._sessions[session.session_id] = session

    def add_query_to_session(self, session_id: str, query_id: str):
        """
        Add a query ID to a session's query list
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                session.queries.append(query_id)
                session.last_accessed = datetime.now()
                self._sessions[session_id] = session

    def get_session_queries(self, session_id: str) -> List[str]:
        """
        Get all query IDs for a session
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                return session.queries.copy()  # Return a copy to prevent external modification
            return []

    def clear_session(self, session_id: str):
        """
        Clear a session's query history
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session:
                session.queries = []
                session.last_accessed = datetime.now()

    def delete_session(self, session_id: str):
        """
        Delete a session
        """
        with self._lock:
            if session_id in self._sessions:
                del self._sessions[session_id]

    def get_all_sessions(self) -> List[ChatSession]:
        """
        Get all active sessions
        """
        with self._lock:
            return list(self._sessions.values())

    def cleanup_old_sessions(self, hours_old: int = 24):
        """
        Remove sessions that haven't been accessed in the specified number of hours
        """
        with self._lock:
            cutoff_time = datetime.now()
            sessions_to_remove = []

            for session_id, session in self._sessions.items():
                if (cutoff_time - session.last_accessed).total_seconds() > hours_old * 3600:
                    sessions_to_remove.append(session_id)

            for session_id in sessions_to_remove:
                del self._sessions[session_id]

            return len(sessions_to_remove)


# Global session manager instance
session_manager = SessionManager()