# Skill: Retrieve Book Context

## Purpose
Retrieve the most relevant textbook content required to answer a user’s question.

This skill enables Retrieval-Augmented Generation (RAG) by querying the vector database
containing embeddings of the Physical AI & Humanoid Robotics textbook.

## Inputs
- question: A natural language question asked by the user
- optional_filters:
  - chapter
  - section

## Outputs
- A list of relevant text chunks from the book
- Each chunk includes:
  - content
  - chapter
  - section

## Behavior
- Convert the question into an embedding
- Query the vector database (Qdrant)
- Return the top-K most relevant chunks
- Do NOT generate answers

## Constraints
- Must not hallucinate or paraphrase content
- Must only return verbatim or lightly trimmed book text

## Used By
- rag_retriever_agent
