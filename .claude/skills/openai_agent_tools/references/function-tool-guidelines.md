# OpenAI Function Tool Guidelines

## What Are Function Tools
Function tools are external functions that OpenAI Agents
can invoke during execution to access real-world data or systems.

## Design Principles
- Deterministic behavior
- No LLM calls inside tools
- Minimal logic
- Clear input/output contracts

## Common Use Cases
- Vector database queries (Qdrant)
- Database access (Neon Postgres)
- Selection-based content extraction
- Chat history persistence

## Anti-Patterns
- Using tools for reasoning
- Mixing tool logic with agent instructions
- Returning free-form text without structure

## Reference
https://openai.github.io/openai-agents-python/
