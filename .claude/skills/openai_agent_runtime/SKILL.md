# Skill: OpenAI Agent Runtime

## Purpose
Provide a standardized runtime for creating, configuring, and executing
OpenAI Agents using the OpenAI Agents SDK.

## Responsibilities
- Instantiate Agent objects
- Configure instructions and model settings
- Apply guardrails for safety and grounding
- Execute agents using the Runner
- Support multi-agent handoffs

## Internals (Abstracted)
- Agent class
- Runner class
- Context injection
- Guardrails
- Agent handoffs

## Constraints
- This skill must not contain business logic
- Business logic is delegated to tools

## Used By
- RAG Orchestrator
- Subagents
