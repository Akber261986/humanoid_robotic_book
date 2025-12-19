# ADR-0001: Web Application Architecture with Frontend-Backend Separation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 001-rag-chatbot
- **Context:** The constitution requires separate 'frontend' and 'backend' directories to enable independent tracking and development of each component. This decision implements a clear separation of concerns between the user interface and the RAG processing backend.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a web application architecture with clear separation between frontend and backend components:

- **Frontend Structure**: Streamlit application in dedicated 'frontend' directory with components, services, and UI logic
- **Backend Structure**: Python services and API endpoints in dedicated 'backend' directory with RAG processing, indexing, and API services
- **Communication**: RESTful API interface between frontend and backend
- **Deployment**: Independent deployment capability for each component while maintaining tight integration for the RAG functionality

## Consequences

### Positive

- Clear separation of concerns enabling independent development workflows
- Independent testing and deployment capabilities for frontend and backend
- Easier maintenance and scaling of individual components
- Compliance with constitutional requirement for separate directory structure
- Better team collaboration with defined boundaries between frontend and backend developers
- Potential for future mobile or other client applications using the same backend API

### Negative

- Increased complexity with additional network calls between components
- Need for API contract management and versioning
- Slightly more complex local development setup requiring both components to run
- Additional infrastructure considerations for deployment
- Potential for version mismatches between frontend and backend if not properly coordinated

## Alternatives Considered

Alternative 1: Monolithic structure with all code in single directory
- Why rejected: Violates constitution requirement for separate frontend and backend directories

Alternative 2: Microservices architecture with multiple backend services
- Why rejected: Over-engineering for this scope; would add unnecessary complexity for a single RAG chatbot feature

Alternative 3: Direct database access from frontend
- Why rejected: Violates separation principles and security best practices; frontend should not directly access vector database

## References

- Feature Spec: specs/001-rag-chatbot/spec.md
- Implementation Plan: specs/001-rag-chatbot/plan.md
- Related ADRs: ADR-0002, ADR-0003
- Evaluator Evidence: specs/001-rag-chatbot/research.md
