# Data Model: RAG-Enabled Gemini Chatbot for Humanoid Robotics Book

**Feature**: 001-rag-chatbot
**Date**: 2025-12-11
**Status**: Completed

## Overview

This document defines the data models for the RAG-enabled Gemini Chatbot, focusing on the entities required to implement the feature while maintaining separation between frontend and backend components.

## Entity: Book Content Chunk

**Description**: Represents a segment of book content that has been processed and stored in the vector database

**Fields**:
- `chunk_id`: Unique identifier for the chunk (string, required)
- `text`: The actual text content of the chunk (string, required)
- `file_path`: Path to the original source file (string, required)
- `heading`: Optional heading if the chunk starts with a section header (string, optional)
- `embedding`: Vector representation of the text content (array of floats, required)
- `metadata`: Additional metadata about the chunk (object, optional)

**Validation Rules**:
- `chunk_id` must be unique across all chunks
- `text` must be between 500-800 characters (as specified in requirements)
- `file_path` must reference an existing file in the docs directory
- `embedding` must be a valid vector representation

**Relationships**:
- None (standalone entity for vector storage)

## Entity: User Query

**Description**: Represents a text input from the user seeking information about humanoid robotics topics

**Fields**:
- `query_id`: Unique identifier for the query (string, required)
- `text`: The actual query text from the user (string, required)
- `timestamp`: When the query was submitted (datetime, required)
- `session_id`: Identifier for the chat session (string, required)

**Validation Rules**:
- `text` must not exceed 1000 characters (as specified in requirements)
- `timestamp` must be current or past time
- `session_id` must be a valid session identifier

**Relationships**:
- References zero or more Book Content Chunks (through retrieval process)

## Entity: Chat Session

**Description**: Represents a persistent conversation context that maintains user query history and system responses

**Fields**:
- `session_id`: Unique identifier for the session (string, required)
- `created_at`: When the session was created (datetime, required)
- `last_accessed`: When the session was last accessed (datetime, required)
- `queries`: List of queries in this session (array of User Query references, optional)

**Validation Rules**:
- `session_id` must be unique
- `created_at` must be before or equal to `last_accessed`
- Session should be cleaned up after inactivity period

**Relationships**:
- Contains multiple User Query entities
- Contains multiple Chat Response entities

## Entity: Chat Response

**Description**: Represents the system's response to a user query, including the generated text and source citations

**Fields**:
- `response_id`: Unique identifier for the response (string, required)
- `query_id`: Reference to the corresponding query (string, required)
- `text`: The generated response text (string, required)
- `sources`: List of source chunks used to generate the response (array of Book Content Chunk references, required)
- `timestamp`: When the response was generated (datetime, required)

**Validation Rules**:
- `text` must be under 300 words (as specified in requirements)
- `sources` must contain at least one valid source chunk
- `query_id` must reference an existing User Query

**Relationships**:
- References one User Query
- References multiple Book Content Chunks

## Entity: API Request

**Description**: Represents an API request to the backend services

**Fields**:
- `request_id`: Unique identifier for the request (string, required)
- `endpoint`: The API endpoint being called (string, required)
- `method`: HTTP method used (string, required)
- `timestamp`: When the request was made (datetime, required)
- `session_id`: Reference to the chat session (string, required)

**Validation Rules**:
- `method` must be a valid HTTP method
- `endpoint` must be a valid API endpoint
- `session_id` must reference an existing session

**Relationships**:
- References one Chat Session