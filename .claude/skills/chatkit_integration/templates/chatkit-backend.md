Create backend endpoints to support OpenAI ChatKit.

Required endpoints:
- Create a ChatKit session
- Receive user messages
- Forward messages to the OpenAI Agent runtime
- Stream responses back to ChatKit

Guidelines:
- Authenticate session creation
- Attach user metadata if available
- Support optional selected_text payloads
