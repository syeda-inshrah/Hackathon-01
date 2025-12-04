<!--
Sync Impact Report:
Version change: (initial) -> 0.1.0
Modified principles:
  - PRINCIPLE_1_NAME -> Content Quality
  - PRINCIPLE_2_NAME -> Technical Stack
  - PRINCIPLE_3_NAME] -> Styling
  - PRINCIPLE_4_NAME -> Markdown/MDX Standard
  - PRINCIPLE_5_NAME -> Testing/Validation
  - PRINCIPLE_6_NAME -> Error Handling (for the content)
Added sections: None
Removed sections: None (retained placeholders for future use)
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs:
  - TODO(SECTION_2_CONTENT): Define additional constraints or requirements.
  - TODO(SECTION_3_CONTENT): Define development workflow or quality gates.
-->
# Teaching Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Content Quality
All book content MUST be clear, technically accurate, and follow the Spec-Kit Plus SDD-RI workflow structure (Constitution -> Specify -> Clarify -> Plan -> Tasks -> Implement). This ensures a consistent, high-quality educational experience.

### II. Technical Stack
The project uses Docusaurus (React/MDX) with TypeScript for any custom components in `src/`. This provides a robust and well-supported platform for interactive technical documentation.

### III. Styling
All styling MUST use Docusaurus's Infima-based CSS for consistency across the book. No custom CSS is allowed unless absolutely necessary, and any deviation MUST be documented with an Architectural Decision Record (ADR) to justify the choice and its implications.

### IV. Markdown/MDX Standard
All documents MUST use YAML frontmatter (title, chapter, lesson) and MDX for embedding interactive React components. Code blocks MUST specify the language (e.g., `bash`, `typescript`) to enable proper syntax highlighting and readability.

### V. Testing/Validation
Any examples (code snippets, prompts) included in the book MUST be verified by the AI companion before inclusion. This means all examples MUST be "testable" and function as described, ensuring accuracy and reliability of the educational material.

### VI. Error Handling (for the content)
All external links within the book content MUST be checked for validity at build time. Any broken links MUST cause the build to fail, ensuring that readers are always directed to active and relevant resources.

## Additional Constraints

TBD: This section is reserved for additional project-specific constraints or requirements not covered by the core principles.

## Development Workflow

TBD: This section will define the development workflow, code review process, and quality gates for the project.

## Governance
This Constitution supersedes all other project practices. Amendments require thorough documentation, approval by stakeholders, and a clear migration plan for any affected systems or content.

All AI interactions with the user that result in a significant output (e.g., code changes, new features, planning discussions, debugging sessions, spec/task/plan creation, multi-step workflows) MUST be recorded in a Prompt History Record (PHR). PHRs MUST be created after every user prompt, capturing the full verbatim input and a concise summary of the assistant's response. PHRs will be routed to `history/prompts/constitution/` for constitution-related interactions, `history/prompts/<feature-name>/` for feature-specific stages, and `history/prompts/general/` for general interactions. Each PHR MUST be uniquely identified and contain complete metadata as per the PHR template.

**Version**: 0.1.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04