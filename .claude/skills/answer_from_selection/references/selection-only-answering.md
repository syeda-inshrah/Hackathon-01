# Selection-Only Answering Policy

## Objective
Guarantee that answers are generated exclusively from user-selected text.

## Constraints
- The selected text is the sole source of truth.
- External knowledge, assumptions, or inference are prohibited.

## Allowed Operations
- Quoting from selected text
- Simple rephrasing without altering meaning

## Disallowed Operations
- Adding missing context
- Generalizing beyond the selection
- Using prior knowledge

## Failure Mode
If the selected text does not contain the answer:
- Respond with:
  "Not found in selected text."

## Why This Matters
This capability ensures precise, user-controlled grounding and directly
fulfills the hackathon requirement for selection-based answering.
