# ChatKit Architecture Reference

## Role of ChatKit
ChatKit provides a prebuilt conversational UI for interacting
with OpenAI-powered agents.

## Architecture
- Frontend: ChatKit widget embedded in Docusaurus
- Backend: FastAPI service
- Intelligence: OpenAI Agents SDK

## Data Flow
1. User sends message via ChatKit UI
2. Backend forwards message to agent runtime
3. Agent executes tools and reasoning
4. Response is streamed back to ChatKit

## Key Principles
- UI is presentation-only
- Backend owns all logic
- Agent runtime is the single source of intelligence

## Reference
https://platform.openai.com/docs/guides/chatkit

