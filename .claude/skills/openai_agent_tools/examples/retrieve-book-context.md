# Example Tool: Retrieve Book Context

## Tool Name
retrieve_book_context

## Purpose
Fetch relevant textbook chunks from the vector database
to support Retrieval-Augmented Generation (RAG).

## Example Implementation (Conceptual)

```python
from agents import function_tool

@function_tool
def retrieve_book_context(query: str, top_k: int = 5) -> list[str]:
    """
    Retrieve relevant textbook chunks from the vector database.
    """
    results = qdrant.search(query=query, limit=top_k)
    return [item.text for item in results]
