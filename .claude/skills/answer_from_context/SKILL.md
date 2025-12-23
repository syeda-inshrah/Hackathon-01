# Skill: Answer From Context

## Purpose
Generate an answer strictly using retrieved textbook context.

## Inputs
- context_chunks: Retrieved textbook content
- question: User query

## Output
- A grounded answer derived exclusively from the context

## Behavior
- Analyze the provided context
- Answer only what is supported by the text
- Use direct explanations from the book

## Constraints
- No external knowledge
- No assumptions
- If information is missing, explicitly say so

## Used By
- grounded_answer_agent
