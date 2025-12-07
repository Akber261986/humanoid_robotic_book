<!--
Sync Impact Report:
Version change: 0.0.0 → 1.0.0 (MAJOR: Initial constitution creation)
Modified principles:
- [PRINCIPLE_1_NAME] → Accuracy
- [PRINCIPLE_2_NAME] → Clarity
- [PRINCIPLE_3_NAME] → Reproducibility
- [PRINCIPLE_4_NAME] → Rigor
- [PRINCIPLE_5_NAME] → Key Standards
Added sections:
- Constraints
- Success Criteria
Removed sections:
- [PRINCIPLE_6_NAME]
- [PRINCIPLE__DESCRIPTION]
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
- TODO(RATIFICATION_DATE): Original adoption date unknown, mark as TODO.
-->
# Educational book on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Accuracy
Accuracy through verification against established robotics frameworks and tools (e.g., ROS 2, Gazebo, NVIDIA Isaac).

### II. Clarity
Clarity for student audience (computer science or engineering background, assuming introductory AI knowledge).

### III. Reproducibility
Reproducibility (all code examples, simulations, and hardware setups must be executable and traceable).

### IV. Rigor
Rigor (prefer official documentation, tutorials, and industry-standard practices).

### V. Key Standards
- All technical claims must be traceable to sources (e.g., official ROS docs, NVIDIA developer resources)
- Citation format: Markdown links or inline references to URLs/docs
- Source types: minimum 50% official documentation and open-source repositories
- Plagiarism check: 0% tolerance; all content must be original synthesis
- Writing clarity: Flesch-Kincaid grade 8-10 for accessibility

## Constraints

- Word count: Flexible per chapter, total book 10,000-20,000 words
- Minimum 10 sources per module (e.g., docs, papers, repos)
- Format: Markdown files for Docusaurus compatibility, with code blocks in Python/ROS syntax

## Success Criteria

- All claims verified against sources and tools
- Zero plagiarism detected
- Passes technical review for reproducibility (e.g., simulations run without errors)
- Content deployable to GitHub Pages via SpeckitPlus and Cloud Code without issues

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan.
All PRs/reviews must verify compliance; Complexity must be justified; Use CLAUDE.md for runtime development guidance.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown | **Last Amended**: 2025-12-07
