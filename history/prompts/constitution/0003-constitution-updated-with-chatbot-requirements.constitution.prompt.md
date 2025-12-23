---
id: 0003
title: Constitution updated with chatbot requirements
stage: constitution
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: hackathon-participant
command: /sp.constitution
labels: ["constitution", "chatbot", "rag", "hackathon", "ai-native-textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Manual validation of constitution structure
---

## Prompt

@"Hackathon I_ Physical AI & Humanoid Robotics Textbook.md" now analyze this file and update the constitution wrt chatbot as well

## Response snapshot

Constitution updated from v1.0.0 to v1.1.0 (MINOR version bump). Added comprehensive RAG chatbot integration requirements and AI-native textbook standards based on hackathon requirements file.

**Key Changes:**
1. **Education-First Design** expanded to include AI-native textbook requirements with embedded chatbot interaction
2. **Integration with Real Robotics Systems** expanded with mandatory RAG chatbot integration
3. Added **RAG Chatbot Integration Standards** section (NON-NEGOTIABLE) covering:
   - Core tech stack: OpenAI ChatKit SDK, FastAPI, Neon Postgres, Qdrant Cloud
   - Functionality requirements: full-book queries, selected-text queries, conversation history, source citations
   - Integration requirements: embedded widget, non-blocking initialization
4. Added **AI-Native Textbook Requirements** section covering:
   - Content vectorization pipeline (chunking, embedding, metadata)
   - Quality standards (reproducibility, relevance validation, source attribution)
5. Added **User Authentication & Personalization** section (OPTIONAL BONUS) covering:
   - Better Auth integration
   - User background collection
   - Personalization features (content depth adjustment, Urdu translation)
6. Added **Technology Stack Standards** section with detailed frontend/backend/robotics stack requirements
7. Expanded **Development Workflow** with chatbot-specific development steps
8. Enhanced **Governance** section with hackathon submission criteria

## Outcome

- ✅ Impact: Constitution now provides clear guidance for building AI-native textbook with embedded RAG chatbot. All hackathon requirements (base + bonus features) documented as constitutional principles.
- 🧪 Tests: N/A (constitutional document)
- 📁 Files: Updated .specify/memory/constitution.md with version bump 1.0.0 → 1.1.0
- 🔁 Next prompts: Update dependent templates (plan, spec, tasks) with chatbot integration checklists
- 🧠 Reflection: Constitution now bridges robotics curriculum content with modern AI-native textbook delivery. Technology choices (Qdrant, Neon, ChatKit) balance free-tier hackathon constraints with production scalability.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitutional amendment followed standard process
- Graders run and results (PASS/FAIL): N/A - manual validation required for governance document
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Create spec.md for chatbot integration feature using updated constitution as reference
