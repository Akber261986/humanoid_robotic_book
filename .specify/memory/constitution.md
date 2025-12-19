<!--
Sync Impact Report:
Version change: 1.1.0 → 1.2.0 (MINOR: Add directory structure requirement)
Modified principles:
- Added directory structure constraint
Added sections:
- Directory organization requirement
Removed sections:
- None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .claude/commands/sp.adr.md: ✅ updated
- .claude/commands/sp.analyze.md: ✅ updated
- .claude/commands/sp.checklist.md: ✅ updated
- .claude/commands/sp.clarify.md: ✅ updated
- .claude/commands/sp.constitution.md: ✅ updated
- .claude/commands/sp.git.commit_pr.md: ✅ updated
- .claude/commands/sp.implement.md: ✅ updated
- .claude/commands/sp.phr.md: ✅ updated
- .claude/commands/sp.plan.md: ✅ updated
- .claude/commands/sp.specify.md: ✅ updated
- .claude/commands/sp.tasks.md: ✅ updated
Follow-up TODOs:
- None
-->
# Humanoid Robotics Book RAG Chatbot Constitution

## Core Principles

### I. User-Centric Focus
Deliver accurate, context-aware responses to user queries about humanoid robotics, drawing directly from book content to enhance learning and hackathon preparation.

### II. Transparency & Reliability
Use Retrieval-Augmented Generation (RAG) to ground responses in verifiable book sources, reducing hallucinations. Always cite retrieved sections.

### III. Privacy & Security
Handle no user data beyond queries; use local Qdrant for vector storage to avoid cloud dependencies unless specified.

### IV. Extensibility
Modular design allowing easy updates to book content, embedding models, or UI. Inspired by simple Gemini bots (e.g., chatkit-gemini-bot), but enhanced with RAG for depth.

### V. Tech Stack Constraints
Python-based (Streamlit for UI, Google Generative AI for Gemini integration).

## Constraints

- All responses must be grounded in book content with proper citations
- No user data storage beyond immediate query processing
- Local vector storage (Qdrant) for privacy compliance
- Python-first approach for all components (backend, processing, UI)
- Modular architecture allowing component replacement without system rewrite
- Directory structure: Separate 'frontend' and 'backend' directories to enable independent tracking and development of each component

## Success Criteria

- Zero hallucinations in responses (all answers backed by book content)
- Fast response times (sub-3 second latency for typical queries)
- Accurate citation of source material with page/section references
- Secure handling of all user queries without data persistence
- Scalable architecture supporting concurrent users during hackathon events

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan.
All PRs/reviews must verify compliance with RAG principles; Complexity must be justified; Use CLAUDE.md for runtime development guidance.

**Version**: 1.2.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-11
