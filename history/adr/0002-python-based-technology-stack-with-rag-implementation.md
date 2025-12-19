# ADR-0002: Python-based Technology Stack with RAG Implementation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 001-rag-chatbot
- **Context:** The constitution requires a Python-based approach with Streamlit for UI and Google Generative AI integration. This decision implements a Retrieval-Augmented Generation pattern to ensure all responses are grounded in book content with proper citations.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a Python-based technology stack with RAG architecture:

- **Language**: Python 3.11+ for all components
- **Frontend**: Streamlit for UI with rapid development and deployment capabilities
- **AI/ML**: Google Generative AI for embeddings and response generation
- **RAG Pipeline**: Retrieval-Augmented Generation pattern with content indexing, similarity search, and citation generation
- **Content Processing**: Markdown parsing for book content processing
- **Environment Management**: python-dotenv for configuration management

## Consequences

### Positive

- Alignment with constitutional requirement for Python-based approach
- Streamlit provides rapid UI development and deployment capabilities
- Google Generative AI offers reliable embeddings and generation services
- RAG pattern ensures responses are grounded in book content with proper citations
- Markdown processing directly supports existing book content format
- Strong ecosystem integration between chosen technologies
- Good developer experience with Python-based tooling

### Negative

- Dependency on Google's API services and potential rate limits
- Learning curve for team members not familiar with Streamlit
- Potential vendor lock-in to Google's ecosystem
- API costs associated with embedding and generation services
- Need for proper error handling and fallback mechanisms for API failures

## Alternatives Considered

Alternative 1: Node.js with Express + LangChain + Pinecone
- Why rejected: Would violate constitution's Python-based requirement; also cloud-based vector storage violates privacy principle

Alternative 2: Go with custom API + PostgreSQL + pgvector
- Why rejected: Would violate constitution's Python-based requirement; Go ecosystem less suitable for AI/ML integration

Alternative 3: Java with Spring Boot + Elasticsearch
- Why rejected: Would violate constitution's Python-based requirement; heavier framework than needed for this use case

## References

- Feature Spec: specs/001-rag-chatbot/spec.md
- Implementation Plan: specs/001-rag-chatbot/plan.md
- Related ADRs: ADR-0001, ADR-0003
- Evaluator Evidence: specs/001-rag-chatbot/research.md
