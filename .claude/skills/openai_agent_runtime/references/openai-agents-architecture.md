# OpenAI Agents Runtime Architecture

This skill follows the OpenAI Agents SDK design:

- Agents encapsulate reasoning and planning
- Tools are invoked via function_tool
- Runner manages execution lifecycle
- Guardrails validate inputs and outputs
- Context is injected for shared resources

Reference:
https://openai.github.io/openai-agents-python/
