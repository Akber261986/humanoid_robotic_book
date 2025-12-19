import streamlit as st
from src.services.chat_service import ChatService
import uuid
import time

# Initialize session state
if "session_id" not in st.session_state:
    st.session_state.session_id = None

if "messages" not in st.session_state:
    st.session_state.messages = []

if "chat_service" not in st.session_state:
    # Use the backend URL - adjust if your backend runs on a different port
    st.session_state.chat_service = ChatService(base_url="http://localhost:8000")

# Set page config
st.set_page_config(
    page_title="Humanoid Robotics Chatbot",
    page_icon="🤖",
    layout="wide"
)

# Initialize session if not already done
if not st.session_state.session_id:
    session_response = st.session_state.chat_service.create_session()
    if "error" not in session_response:
        st.session_state.session_id = session_response.get("session_id", str(uuid.uuid4()))
    else:
        st.error("Failed to create session. Please try again.")
        st.session_state.session_id = str(uuid.uuid4())  # Fallback

# Sidebar
with st.sidebar:
    st.title("🤖 Humanoid Robotics Assistant")
    st.markdown("""
    This chatbot helps you explore the **Humanoid Robotics Book**.

    Ask questions about:
    - Humanoid kinematics
    - AI integration
    - Robot control systems
    - Hackathon projects
    - And more!
    """)

    # Session info
    st.subheader("Session Info")
    st.write(f"Session ID: {st.session_state.session_id}")

    # Clear chat button
    if st.button("🔄 Clear Chat", type="secondary"):
        st.session_state.messages = []
        # Optionally, we could also clear the session on the backend
        # For now, we'll just create a new session to clear context
        session_response = st.session_state.chat_service.create_session()
        if "error" not in session_response:
            st.session_state.session_id = session_response.get("session_id", str(uuid.uuid4()))
        else:
            st.session_state.session_id = str(uuid.uuid4())  # Fallback
        st.rerun()

    # Session details button
    if st.button("📋 Session Details", type="secondary"):
        session_details = st.session_state.chat_service.get_session_details(st.session_state.session_id)
        if "error" not in session_details:
            st.success(f"Session created: {session_details.get('created_at', 'N/A')}")
            st.info(f"Last accessed: {session_details.get('last_accessed', 'N/A')}")
            st.info(f"Queries in session: {session_details.get('query_count', 0)}")
        else:
            st.error("Could not retrieve session details")

    # Health check with refresh
    st.subheader("System Status")
    health = st.session_state.chat_service.health_check()
    if "error" not in health:
        status = health.get("status", "unknown")
        qdrant_status = health.get("dependencies", {}).get("qdrant", "unknown")
        gemini_status = health.get("dependencies", {}).get("gemini_api", "unknown")

        # Display status with icons
        if status == "healthy":
            st.success(f"✅ Service: {status}")
        else:
            st.error(f"❌ Service: {status}")

        if qdrant_status == "connected":
            st.success(f"✅ Qdrant: {qdrant_status}")
        else:
            st.error(f"❌ Qdrant: {qdrant_status}")

        if gemini_status == "available":
            st.success(f"✅ Gemini API: {gemini_status}")
        else:
            st.error(f"❌ Gemini API: {gemini_status}")
    else:
        st.error("⚠️ Could not connect to backend")

# Main content
st.title("🤖 Humanoid Robotics Chatbot")
st.markdown("Ask me anything about humanoid robotics based on the book content!")

# Show welcome message if no messages yet
if not st.session_state.messages:
    with st.chat_message("assistant"):
        st.markdown("Hello! I'm your humanoid robotics assistant. I can answer questions based on the Humanoid Robotics Book. Ask me anything!")

# Display chat messages
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])
        if "sources" in message and message["sources"]:
            with st.expander("📚 Sources"):
                for i, source in enumerate(message["sources"]):
                    st.markdown(f"**Source {i+1}:** {source['file_path']}")
                    st.text_area(
                        f"Content {i+1}",
                        value=source['text'][:500] + "..." if len(source['text']) > 500 else source['text'],
                        height=100,
                        key=f"source_{i}_{message.get('timestamp', 0)}"
                    )

# Chat input
if prompt := st.chat_input("Ask about humanoid robotics..."):
    # Add user message to history
    st.session_state.messages.append({"role": "user", "content": prompt})

    # Display user message
    with st.chat_message("user"):
        st.markdown(prompt)

    # Display assistant response
    with st.chat_message("assistant"):
        message_placeholder = st.empty()
        # Add more visual feedback
        with st.spinner("Processing your query..."):
            # Get response from backend
            response = st.session_state.chat_service.query(prompt, st.session_state.session_id)

            if "error" in response:
                error_msg = f"Sorry, I encountered an error: {response['error']}"
                st.error(error_msg)
                st.session_state.messages.append({
                    "role": "assistant",
                    "content": error_msg,
                    "sources": []
                })
            else:
                # Extract response content and sources
                response_text = response.get("response", "I couldn't find an answer to your question.")
                sources = response.get("sources", [])

                # Display the response
                message_placeholder.markdown(response_text)

                # Add to message history
                st.session_state.messages.append({
                    "role": "assistant",
                    "content": response_text,
                    "sources": sources
                })

                # Show sources if available
                if sources:
                    with st.expander("📚 Sources Used"):
                        for i, source in enumerate(sources):
                            st.markdown(f"**Source {i+1}:** {source['file_path']}")
                            st.text_area(
                                f"Content {i+1}",
                                value=source['text'][:500] + "..." if len(source['text']) > 500 else source['text'],
                                height=100,
                                key=f"final_source_{i}_{time.time()}"
                            )

# Footer
st.markdown("---")
st.markdown(
    "<div style='text-align: center; color: gray; font-size: 0.9em;'>"
    "Powered by Retrieval-Augmented Generation (RAG) with Google Gemini<br>"
    "All responses are grounded in the Humanoid Robotics Book content"
    "</div>",
    unsafe_allow_html=True
)