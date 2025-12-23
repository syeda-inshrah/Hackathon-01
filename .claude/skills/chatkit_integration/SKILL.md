# Skill: ChatKit Integration

## Purpose
Integrate OpenAI ChatKit as the user interface for the RAG chatbot
and connect it to the backend OpenAI Agents runtime.

This skill defines how chat sessions, messages, and streaming
responses flow between the frontend (ChatKit) and backend (FastAPI).

## Responsibilities
- Create ChatKit sessions securely
- Handle ChatKit message events
- Forward user messages to OpenAI Agents
- Stream agent responses back to ChatKit UI
- Support selection-based queries

## What This Skill Covers
- ChatKit session creation
- Backend API endpoints for ChatKit
- Message forwarding to agent runtime
- Streaming responses

## What This Skill Does NOT Cover
- Agent reasoning logic
- Tool definitions
- RAG implementation details

## Constraints
- ChatKit must never call the LLM directly
- All intelligence must reside in the backend agent
- UI must remain stateless

## Used By
- Docusaurus frontend
- FastAPI backend
