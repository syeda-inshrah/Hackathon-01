# Skill: Answer From Selection

## Purpose
Answer a question strictly using user-selected text from the book.

## Inputs
- selected_text: Highlighted book content
- question: User query

## Output
- Answer derived only from selected_text

## Behavior
- Analyze selected text
- Respond only if answer exists in the text

## Constraints
- No external knowledge
- No inference beyond selected text
- Must refuse if answer is not present

## Used By
- grounded_answer_agent
