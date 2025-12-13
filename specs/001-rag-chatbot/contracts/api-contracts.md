# API Contracts: RAG-Enabled Gemini Chatbot for Humanoid Robotics Book

**Feature**: 001-rag-chatbot
**Date**: 2025-12-11
**Status**: Draft

## Overview

This document defines the API contracts for the backend services of the RAG-enabled Gemini Chatbot. These contracts establish the interface between the frontend and backend components as required by the constitution's separation of concerns.

## Base URL

All API endpoints are relative to: `http://localhost:8000` (or the deployed backend URL)

## Common Headers

All requests must include:
- `Content-Type: application/json`
- `Accept: application/json`

## Endpoints

### 1. Query Processing

#### POST /api/query

Process a user query and return a RAG-enhanced response with citations.

**Request**:
```json
{
  "query": "Explain inverse kinematics in humanoids",
  "session_id": "session-12345"
}
```

**Request Parameters**:
- `query`: The user's question about humanoid robotics (string, required, max 1000 chars)
- `session_id`: The chat session identifier (string, required)

**Response (200 OK)**:
```json
{
  "response": "Inverse kinematics in humanoid robotics refers to...",
  "sources": [
    {
      "text": "Inverse kinematics (IK) is a mathematical process...",
      "file_path": "docs/kinematics.md",
      "chunk_id": "chunk-123"
    }
  ],
  "query_id": "query-67890",
  "response_id": "response-111213"
}
```

**Response Parameters**:
- `response`: The AI-generated answer to the query (string)
- `sources`: Array of source chunks used to generate the response (array of objects)
- `query_id`: Unique identifier for the query (string)
- `response_id`: Unique identifier for the response (string)

**Error Responses**:
- `400 Bad Request`: Invalid query format or missing parameters
- `500 Internal Server Error`: Processing error or API failure

### 2. Content Indexing

#### POST /api/index

Index the book content from Markdown files into the vector database.

**Request**:
```json
{
  "docs_path": "./docs"
}
```

**Request Parameters**:
- `docs_path`: Path to the directory containing Markdown files (string, required)

**Response (200 OK)**:
```json
{
  "status": "completed",
  "chunks_indexed": 150,
  "files_processed": 12
}
```

**Error Responses**:
- `400 Bad Request`: Invalid path or missing parameters
- `500 Internal Server Error`: Indexing error

### 3. Session Management

#### POST /api/session

Create a new chat session.

**Request**: Empty body

**Response (200 OK)**:
```json
{
  "session_id": "session-98765",
  "created_at": "2025-12-11T10:30:00Z"
}
```

#### GET /api/session/{session_id}

Get session details.

**Path Parameters**:
- `session_id`: The session identifier (string, required)

**Response (200 OK)**:
```json
{
  "session_id": "session-98765",
  "created_at": "2025-12-11T10:30:00Z",
  "last_accessed": "2025-12-11T10:35:00Z",
  "query_count": 5
}
```

**Error Responses**:
- `404 Not Found`: Session does not exist
- `500 Internal Server Error`: Server error

### 4. Health Check

#### GET /health

Check the health status of the backend service.

**Response (200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-11T10:30:00Z",
  "dependencies": {
    "qdrant": "connected",
    "gemini_api": "available"
  }
}
```

## Error Format

All error responses follow this format:

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Additional error details (optional)"
  }
}
```

## Authentication

No authentication required for this API. All endpoints are accessible without authentication, in line with the privacy and security principles of processing queries without storing user data beyond immediate processing.

## Rate Limiting

The API implements rate limiting to prevent abuse:
- Maximum 10 requests per minute per IP address
- 429 Too Many Requests response when limit is exceeded