# Example: ChatKit Embedded in Docusaurus

## Scenario
Embed ChatKit inside the published textbook so users can ask
questions about the chapter they are reading.

## Behavior
- Floating chat widget appears on every page
- User sends a question
- Backend routes the message to the RAG agent
- Answer is returned and displayed in ChatKit

## Special Case: Selected Text
- User highlights text in the chapter
- UI sends `selected_text` along with the message
- Backend routes to selection-based answering
