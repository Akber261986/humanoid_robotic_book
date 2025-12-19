# ADR-0003: Qdrant Vector Database for Local RAG Storage

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 001-rag-chatbot
- **Context:** The constitution requires local vector storage for privacy compliance to avoid cloud dependencies unless specified. This decision implements Qdrant as the vector database for efficient similarity search in the RAG pipeline.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement Qdrant as the vector database for local RAG storage:

- **Database**: Qdrant vector database running locally
- **Deployment**: Docker container for local deployment capability
- **Integration**: Python client library for seamless integration with the Python-based stack
- **Search**: Efficient similarity search with cosine similarity for RAG retrieval
- **Storage**: Local storage to maintain privacy and security requirements

## Consequences

### Positive

- Compliance with constitutional privacy & security principles by using local storage
- Efficient similarity search capabilities optimized for vector data
- Good Python integration with official qdrant-client library
- Suitable performance for expected data volume and query patterns
- Self-hosted solution with no external dependencies for vector storage
- Docker-based deployment for easy local and cloud deployment

### Negative

- Additional infrastructure component requiring Docker or local installation
- Potential operational complexity for deployment compared to managed services
- Need to handle backup and recovery procedures for local storage
- May require additional monitoring and maintenance compared to cloud solutions
- Learning curve for team members unfamiliar with Qdrant

## Alternatives Considered

Alternative 1: Pinecone (cloud-based vector database)
- Why rejected: Would violate privacy principle by using cloud storage; violates constitution requirement for local storage

Alternative 2: Chroma (local vector database)
- Why rejected: Less performant for production use cases compared to Qdrant; less mature for production deployments

Alternative 3: PostgreSQL with pgvector extension
- Why rejected: Not optimized specifically for vector search operations; Qdrant provides better performance for similarity search

## References

- Feature Spec: specs/001-rag-chatbot/spec.md
- Implementation Plan: specs/001-rag-chatbot/plan.md
- Related ADRs: ADR-0001, ADR-0002
- Evaluator Evidence: specs/001-rag-chatbot/research.md
