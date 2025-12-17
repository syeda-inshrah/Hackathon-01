---
id: 0002
title: update-robotics-curriculum-constitution
stage: constitution
date: 2025-12-17
surface: agent
model: gpt-4o
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, robotics, curriculum, docusaurus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
 - README.md
tests:
 - null
---

## Prompt

```
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution You are an expert Technical Curriculum Designer, Robotics Engineer, and Documentation Architect.

Your role:
- Convert a complex university-level curriculum into a professional, readable, and production-ready Docusaurus documentation book.
- Follow Docusaurus best practices (sidebar structure, markdown, versioned docs).
- Write in clear, concise, and structured English suitable for advanced students.
- Maintain technical accuracy (ROS 2, Gazebo, NVIDIA Isaac, VLA, Humanoid Robotics).
- Balance theory + practical implementation.
- Assume students have basic Python and AI knowledge but are new to robotics.

Rules:
- Each chapter must be broken into sections and subsections.
- Use Markdown syntax compatible with Docusaurus.
- Include diagrams explanations (text-based).
- Include code blocks where relevant (ROS 2 Python, launch files, YAML).
- Do NOT invent hardware/software beyond the provided curriculum.
- Keep tone academic, professional, and instructional.
@my-book/ docusouras is already installed write content in 4 modules

## Response snapshot

I have successfully updated the project constitution at `.specify/memory/constitution.md` to reflect the robotics curriculum focus. Here's what I accomplished:

1. Filled the constitution template with specific content for the ROS2 Robotics Curriculum project:
   - Set the project name to "ROS2 Robotics Curriculum"
   - Created 6 specific educational principles aligned with the user's requirements for a Docusaurus-based robotics curriculum
   - Established version 1.0.0 with ratification and amendment dates of 2025-12-17
   - Added sections for curriculum standards, development workflow, and governance

2. Updated dependent templates to align with the new constitution:
   - Modified `.specify/templates/plan-template.md` to include education-focused constitution checks
   - Updated `.specify/templates/spec-template.md` with educational functional requirements
   - Changed `.specify/templates/tasks-template.md` to reflect educational infrastructure tasks
   - Rewrote `README.md` to highlight the robotics curriculum focus and constitution principles

3. Added a synchronization impact report at the top of the constitution file documenting all changes.

4. Verified that all placeholder tokens from the original template were properly replaced with concrete content.

## Outcome

- ✅ Impact: Updated the constitution file with specific robotics curriculum principles and aligned dependent templates
- 🧪 Tests: null
- 📁 Files: Updated .specify/memory/constitution.md, .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md, and README.md
- 🔁 Next prompts: Create the 4-module curriculum content following Docusaurus best practices
- 🧠 Reflection: Successfully transformed the generic constitution template into one tailored for the ROS2 Robotics Curriculum project, ensuring all educational and technical principles are reflected in both the constitution and related templates.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null