# Tasks: RAG-Enabled Gemini Chatbot for Humanoid Robotics Book

**Feature**: 001-rag-chatbot
**Generated**: 2025-12-11
**Based on**: specs/001-rag-chatbot/

## Implementation Strategy

This plan implements the RAG-enabled chatbot incrementally, starting with the core functionality (User Story 1) as an MVP, then adding supporting features. Each user story forms a complete, independently testable increment.

- **MVP Scope**: User Story 1 (Core query functionality) with basic indexing and query handling
- **Testing Strategy**: Manual testing for UI components; unit tests for backend functions as needed
- **Parallel Opportunities**: Backend service development can run in parallel with frontend implementation once contracts are established

## Dependencies

- User Story 1 (P1) → User Story 2 (P2) → User Story 3 (P3)
- Foundational tasks must complete before user stories begin

## Parallel Execution Examples

- T006-T010 [P] - Backend services can be developed in parallel
- T012-T014 [P] - Frontend components can be developed in parallel
- T015-T017 [P] - API endpoints can be developed in parallel

---

## Phase 1: Setup

### Goal
Initialize project structure with proper directory separation and dependencies.

### Tasks

- [x] T001 Create project directory structure (backend/ and frontend/ directories)
- [x] T002 Create backend requirements.txt with streamlit, google-generativeai==0.8.3, qdrant-client==1.11.0, markdown==3.7, python-dotenv
- [x] T003 Create frontend requirements.txt with streamlit, requests
- [x] T004 Create .env file template with GEMINI_API_KEY and QDRANT_URL variables
- [x] T005 Create .gitignore for Python project with virtual environments and sensitive files

---

## Phase 2: Foundational

### Goal
Establish core infrastructure and common utilities needed for all user stories.

### Tasks

- [x] T006 Create backend/src/models directory and BookContentChunk data class in backend/src/models/chunk.py
- [x] T007 Create backend/src/models/UserQuery data class in backend/src/models/query.py
- [x] T008 Create backend/src/models/ChatSession data class in backend/src/models/session.py
- [x] T009 Create backend/src/models/ChatResponse data class in backend/src/models/response.py
- [x] T010 Create backend/src/services/config.py with configuration loading from environment variables
- [x] T011 Create backend/src/services/vector_store.py with Qdrant connection and basic operations
- [x] T012 Create backend/src/services/gemini_client.py with Gemini API integration for embeddings and generation
- [x] T013 Create backend/src/utils/chunking.py with text chunking functions (500-800 chars)
- [x] T014 Create backend/src/utils/markdown_parser.py with Markdown file parsing functions

---

## Phase 3: User Story 1 - Query Humanoid Robotics Knowledge (Priority: P1)

### Goal
Enable users to ask questions about humanoid robotics and receive accurate responses with citations.

### Independent Test Criteria
Can be fully tested by asking specific questions about humanoid robotics and verifying that the system returns relevant, accurate answers with proper citations to book content.

### Tasks

- [x] T015 [US1] Implement content indexing functionality in backend/src/services/indexer.py
- [x] T016 [US1] Create RAG pipeline service in backend/src/services/rag.py with embed_query, retrieve, augment_prompt, generate_response functions
- [x] T017 [US1] Create API endpoint POST /api/query in backend/src/api/main.py
- [x] T018 [P] [US1] Create frontend/src/services/chat_service.py with API communication functions
- [x] T019 [P] [US1] Create Streamlit frontend app in frontend/app.py with basic query interface
- [x] T020 [US1] Implement content chunking with 500-800 character limits in backend/src/utils/chunking.py
- [x] T021 [US1] Test basic query functionality with sample humanoid robotics questions
- [x] T022 [US1] Implement error handling for "No relevant information found" scenarios

---

## Phase 4: User Story 2 - View Source Citations (Priority: P2)

### Goal
Allow users to verify information by viewing original book content used in responses.

### Independent Test Criteria
Can be tested by asking questions that generate responses with citations, then expanding those citations to view the original book content.

### Tasks

- [x] T023 [US2] Enhance API response to include chunk_id and file_path in sources array
- [x] T024 [US2] Update Streamlit UI in frontend/app.py to display citations as expandable elements
- [x] T025 [US2] Implement citation expansion functionality in frontend/app.py
- [x] T026 [US2] Create function to retrieve original content by chunk_id in backend/src/services/rag.py
- [x] T027 [US2] Test citation display and expansion with multiple query examples

---

## Phase 5: User Story 3 - Persistent Chat Session (Priority: P3)

### Goal
Maintain conversation context across multiple questions within a session.

### Independent Test Criteria
Can be tested by asking multiple related questions in sequence and verifying that the chat history is maintained and accessible.

### Tasks

- [ ] T028 [US3] Implement session management in backend/src/services/session_manager.py
- [ ] T029 [US3] Add session creation endpoint POST /api/session in backend/src/api/main.py
- [ ] T030 [US3] Add session details endpoint GET /api/session/{session_id} in backend/src/api/main.py
- [ ] T031 [US3] Update query endpoint to maintain session context in backend/src/api/main.py
- [ ] T032 [US3] Implement session state in frontend/app.py using Streamlit's session state
- [ ] T033 [US3] Test multi-turn conversations with session persistence
- [ ] T034 [US3] Implement clear/reset session functionality in frontend/app.py

---

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Address edge cases, performance, error handling, and deployment concerns.

### Tasks

- [ ] T035 Implement query truncation for inputs over 1000 characters in backend/src/services/rag.py
- [ ] T036 Add retry logic with exponential backoff for Gemini API calls in backend/src/services/gemini_client.py
- [ ] T037 Implement rate limiting for API endpoints in backend/src/api/main.py
- [ ] T038 Add comprehensive error handling and logging in backend/src/services/rag.py
- [ ] T039 Create health check endpoint GET /health in backend/src/api/main.py
- [ ] T040 Add loading indicators and better UX in frontend/app.py
- [ ] T041 Implement empty database handling with clear user instructions in backend/src/services/indexer.py
- [ ] T042 Create Dockerfile for backend in backend/Dockerfile
- [ ] T043 Create Dockerfile for frontend in frontend/Dockerfile
- [ ] T044 Update quickstart documentation in README.md
- [ ] T045 Perform end-to-end testing with 5 sample book queries
- [ ] T046 Verify 3-second response time requirement is met