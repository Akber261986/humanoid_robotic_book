# ADR-0001: Technical Stack and Conceptual Data Model for Physical AI Robotics Book

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Proposed
- **Date:** 2025-12-07
- **Feature:** 001-physical-ai-robotics-book
- **Context:** The goal is to create an educational book on Physical AI and Humanoid Robotics for university students and early-career engineers. This decision outlines the core technical stack and conceptual data model to ensure a consistent, practical, and up-to-date learning experience, leveraging industry-standard tools like ROS 2, Gazebo, and NVIDIA Isaac Sim.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

*   **Python Version**: Python 3.11
*   **Primary Dependencies**: ROS 2, Gazebo, NVIDIA Isaac Sim, OpenVLA, Isaac Lab, Isaac ROS 2 packages (e.g., Nvblox, Object Detection, DNN Inference, Visual SLAM, Common), NumPy, SciPy, OpenCV, PyTorch/TensorFlow, Scikit-learn, Matplotlib/Seaborn, Pandas.
*   **Storage**: N/A (educational content, no application data storage)
*   **Testing**: `pytest` (primary), `unittest` (alternative), with a focus on modular design, mocking, and simulation independence.
*   **Target Platform**: Linux (Ubuntu 20.04+ recommended).
*   **Project Type**: Educational Content / Technical Documentation.
*   **Content Format**: Markdown source with inline links to sources/docs, Python/ROS syntax code blocks.
*   **Conceptual Data Model**: Defined key entities (Physical AI, Humanoid Robotics, ROS 2, Gazebo, NVIDIA Isaac Sim, VLA Integration) and their relationships, serving as a structured understanding of book concepts.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

*   **Industry Relevance**: Aligns with current industry standards and tools (ROS 2, NVIDIA Isaac Sim), providing practical skills.
*   **Seamless Integration**: Python 3.11 ensures compatibility with NVIDIA Isaac Sim and its internal ROS capabilities.
*   **Comprehensive Coverage**: A broad selection of scientific and AI libraries supports diverse topics in robotics (perception, control, ML).
*   **Reproducibility**: Emphasis on testable, reproducible examples in simulation, aligning with constitutional principles.
*   **Structured Learning**: The conceptual data model provides a clear, organized framework for the book's content.

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

*   **Version Dependency**: Strong dependency on Python 3.11 for Isaac Sim might require environment management for users with different Python setups.
*   **Ecosystem Specificity**: While leveraging NVIDIA's ecosystem offers benefits, it might limit exposure to other robotics platforms or AI frameworks.
*   **Dynamic Nature of AI/Robotics**: VLA integration frameworks and AI libraries are rapidly evolving, potentially requiring frequent updates to content.
*   **Learning Curve**: The breadth of tools (ROS 2, Gazebo, Isaac Sim, various Python libraries) presents a significant learning curve for the target audience, necessitating clear pedagogical approaches.

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

*   **Alternative Python Version**: Standard Python 3.10 (often default with ROS 2 Humble) or Python 3.12 (for newer ROS 2 Jazzy). *Rejected because*: Python 3.11 is explicitly required for NVIDIA Isaac Sim's internal operations, which is a core tool for this book. Using other versions would complicate the integration and might lead to compatibility issues with key simulation tools.
*   **Alternative VLA Integration**: Relying solely on general-purpose LLM/VLM APIs (e.g., direct OpenAI API, Hugging Face `transformers` without NVIDIA-specific optimizations). *Rejected because*: NVIDIA's OpenVLA and Isaac Lab/ROS packages offer more direct and optimized integrations for embodied robotics, including sim2real capabilities, which are central to the book's practical focus.
*   **Alternative Testing Frameworks**: Exclusive use of `unittest`. *Rejected because*: `pytest` offers more flexibility, a simpler syntax, and a more powerful fixture system for managing the complex setups common in robotics, making test development and maintenance more efficient.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: /mnt/d/GIAIC/Hackathon/humanoid_robotic_book/specs/001-physical-ai-robotics-book/spec.md
- Implementation Plan: /mnt/d/GIAIC/Hackathon/humanoid_robotic_book/specs/001-physical-ai-robotics-book/plan.md
- Related ADRs: N/A
- Evaluator Evidence: N/A <!-- link to eval notes/PHR showing graders and outcomes -->
