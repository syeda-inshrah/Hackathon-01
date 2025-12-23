<!-- SYNC IMPACT REPORT
Version change: 1.0.0 -> 1.1.0
Modified principles:
  - Education-First Design (expanded to include AI-native textbook requirements)
  - Integration with Real Robotics Systems (expanded with chatbot integration)
Added sections:
  - RAG Chatbot Integration Standards
  - AI-Native Textbook Requirements
  - User Authentication & Personalization (optional bonus features)
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending (add chatbot integration checklist)
  - .specify/templates/spec-template.md ⚠ pending (add RAG requirements section)
  - .specify/templates/tasks-template.md ⚠ pending (add chatbot task categories)
  - .specify/templates/commands/*.md ⚠ pending
Runtime guidance docs:
  - README.md ⚠ pending (add chatbot deployment instructions)
  - Hackathon requirements integrated
Follow-up TODOs:
  - Update plan template with chatbot architecture checklist
  - Document OpenAI ChatKit SDK integration patterns
  - Add Qdrant vector DB setup guide
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Education-First Design
Every curriculum module MUST start with clear learning objectives. Content MUST be self-contained, independently understandable, and well-documented. Clear pedagogical purpose required - no sections without educational value. All content MUST be designed as an AI-native textbook where readers can interact with embedded AI chatbots for clarification, examples, and personalized learning paths.

**Rationale**: Students learn best when they can ask questions in context and receive immediate, contextual assistance. AI-native textbooks transform passive reading into active learning dialogues.

### Practical Implementation Focus
Every theoretical concept includes hands-on exercises and practical examples; Students learn ROS 2, Gazebo, NVIDIA Isaac, and humanoid robotics through direct implementation; Support both beginner-level tutorials and advanced challenges.

### Test-Driven Learning (NON-NEGOTIABLE)
Every lesson includes pre-defined learning outcomes that must be validated; Students demonstrate understanding through practical assessments before progressing; Follow Understand-Apply-Evaluate cycle strictly enforced.

### Integration with Real Robotics Systems
Focus areas requiring integration with real-world robotics frameworks: ROS 2 communication patterns, Simulation environments (Gazebo), NVIDIA Isaac platforms, Humanoid robot control systems. Every textbook deployment MUST include an embedded RAG (Retrieval-Augmented Generation) chatbot capable of answering questions about the book's content and selected text.

**Rationale**: Physical AI requires bridging digital and physical worlds. Similarly, AI-native textbooks bridge static content and dynamic assistance, allowing students to query specific concepts or ask "what if" scenarios grounded in the actual curriculum.

### Modularity and Accessibility
Text-based documentation ensures accessibility; Structured content with progressive complexity; Modules designed to be taken independently for flexible learning paths.

### Docusaurus Best Practices
All content MUST follow Docusaurus standards for navigation, sidebar structure, versioned documentation, and responsive design. The site MUST be deployable to GitHub Pages or Vercel with zero manual configuration after build.

**Rationale**: Docusaurus provides the static site foundation; automated deployment ensures reproducibility and eliminates deployment friction for hackathon submissions.

## RAG Chatbot Integration Standards (NON-NEGOTIABLE)

### Core Chatbot Requirements
The embedded chatbot MUST use:
- **OpenAI Agents/ChatKit SDKs** for conversation management and streaming responses
- **FastAPI** as the backend service layer for chatbot endpoints
- **Neon Serverless Postgres** for persistent conversation history and user context
- **Qdrant Cloud Free Tier** for vector storage and semantic retrieval of textbook content

### Chatbot Functionality
The chatbot MUST support:
1. **Full-book queries**: Answer questions based on the entire textbook corpus
2. **Selected-text queries**: Allow users to highlight text and ask questions specifically about that selection
3. **Conversation history**: Maintain context across multiple turns within a session
4. **Source citations**: Provide references to specific chapters/sections when answering

### Integration Requirements
- Chatbot widget MUST be embedded in every Docusaurus page
- Widget MUST not interfere with navigation or reading experience
- Initial page load MUST NOT be blocked by chatbot initialization
- Chatbot backend MUST be deployed and publicly accessible alongside the static site

**Rationale**: RAG-based chatbots ground responses in actual textbook content, preventing hallucinations. OpenAI's ChatKit provides production-ready streaming UI. FastAPI enables rapid Python-based API development. Neon and Qdrant offer free tiers suitable for hackathon demos while being production-scalable.

## AI-Native Textbook Requirements

### Content Vectorization
All textbook chapters MUST be:
- Chunked into semantically meaningful segments (typically 500-1000 tokens)
- Embedded using OpenAI's `text-embedding-3-small` or equivalent model
- Stored in Qdrant with metadata (chapter, section, page number equivalents)

### Quality Standards
- Embedding pipeline MUST be reproducible (version-controlled scripts)
- Vector search MUST return relevant results (manual spot-check minimum 10 queries)
- Chatbot responses MUST cite source sections with clickable links back to content

**Rationale**: Proper chunking ensures retrieval precision. Metadata enables rich source attribution. Reproducibility ensures the textbook can be updated and re-indexed systematically.

## User Authentication & Personalization (OPTIONAL BONUS)

### Authentication Stack
If implementing authentication (bonus points), MUST use:
- **Better Auth** (better-auth.com) for signup/signin flows
- Neon Postgres for user profile storage
- Secure token management (JWT or session-based)

### User Background Collection
At signup, MUST collect:
- Software background (beginner/intermediate/advanced in Python, ROS, AI)
- Hardware background (access to GPUs, Jetson kits, robots)

### Personalization Features
If implemented, MUST support:
1. **Content personalization**: Button at chapter start to adjust technical depth based on user background
2. **Translation**: Button at chapter start to translate content to Urdu
3. **Persistent preferences**: User settings saved across sessions

**Rationale**: Personalization increases accessibility and engagement. Better Auth simplifies implementation. Background data enables adaptive content without manual configuration per session.

## Technology Stack Standards

### Frontend Stack (NON-NEGOTIABLE)
- **Static Site Generator**: Docusaurus v3+ with React 18+
- **Deployment Targets**: GitHub Pages OR Vercel (MUST work with zero manual config)
- **Chatbot UI**: OpenAI ChatKit SDK embedded as React component

### Backend Stack (NON-NEGOTIABLE)
- **API Framework**: FastAPI (Python 3.10+)
- **Vector Database**: Qdrant Cloud Free Tier
- **Relational Database**: Neon Serverless Postgres (free tier)
- **Embedding Model**: OpenAI `text-embedding-3-small` or equivalent
- **LLM**: OpenAI GPT-4 or GPT-4-turbo via OpenAI Agents SDK

### Optional Authentication Stack (BONUS)
- **Auth Provider**: Better Auth (better-auth.com)
- **Session Storage**: Neon Postgres
- **Translation**: OpenAI API with prompt engineering for Urdu translation

### Robotics Content Stack
- **ROS Version**: ROS 2 Humble Hawksbill (LTS)
- **Simulation**: Gazebo Classic or Gazebo Sim (Fortress+)
- **AI Platform**: NVIDIA Isaac Sim/ROS (documentation references)
- **Hardware References**: Jetson Orin Nano, Intel RealSense D435i, Unitree robots

**Rationale**: Technology choices balance hackathon constraints (free tiers, rapid development) with production viability (scalable services, industry-standard frameworks).

## Development Workflow

### Content Development
1. All curriculum content MUST be written in Markdown with Docusaurus frontmatter
2. Code examples MUST be executable and tested (ROS 2 launch files, Python scripts)
3. Diagrams MUST use Mermaid or SVG (no proprietary formats)
4. Technical accuracy MUST be verified by robotics engineer before merge

### Chatbot Development
1. Embedding pipeline MUST run before deployment (CI/CD step)
2. Vector database MUST be seeded with textbook content before launch
3. Backend API MUST have health checks (`/health`, `/ready` endpoints)
4. ChatKit integration MUST handle errors gracefully (fallback messages)

### Quality Gates
- All content MUST pass Docusaurus build without warnings
- Chatbot MUST respond to 10 test queries with correct source citations
- Deployment MUST succeed on target platform (GitHub Pages/Vercel)
- Optional: Authentication flow MUST complete signup→login→personalization cycle

## Governance

### Constitutional Authority
This Constitution supersedes all other development practices. Any code, content, or architecture decisions MUST align with the principles defined herein.

### Amendment Process
Amendments require:
1. Documentation of educational or technical impact justification
2. Approval by both technical lead (robotics/AI accuracy) and educational lead (pedagogy)
3. Version bump following semantic versioning (major: principle changes, minor: new sections, patch: clarifications)

### Compliance Enforcement
- All PRs MUST be reviewed for compliance with both technical and educational standards
- Content complexity MUST be justified by explicit learning objectives
- Chatbot integration MUST pass functional tests before deployment
- Optional features (auth, personalization, translation) MUST NOT break core functionality

### Hackathon Submission Criteria
Final submission MUST include:
1. Public GitHub repository with complete source code
2. Deployed book accessible via GitHub Pages or Vercel
3. Functional embedded RAG chatbot with backend API
4. Demo video under 90 seconds demonstrating core features
5. README documenting setup, architecture, and bonus features implemented

**Rationale**: Clear governance ensures consistent decision-making under time pressure. Hackathon criteria anchor development priorities (core functionality first, bonuses second).

---

**Version**: 1.1.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-22