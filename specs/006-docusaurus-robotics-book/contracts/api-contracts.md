# API Contracts: Docusaurus Robotics Book

## Overview
This document defines the API contracts for any backend services that might support the Docusaurus-based robotics curriculum site. This includes user progress tracking, assessment submissions, and potentially interactive simulation interfaces.

## Assessment Submission API

### Submit Assessment Response
- **Endpoint**: `POST /api/assessments/submit`
- **Description**: Submit responses to a curriculum assessment
- **Authentication**: Required (JWT token)

**Request Body**:
```json
{
  "assessmentId": "string",
  "userId": "string",
  "moduleId": "string",
  "responses": [
    {
      "questionId": "string",
      "answer": "string or number or array"
    }
  ],
  "timestamp": "ISO 8601 datetime"
}
```

**Response (200 OK)**:
```json
{
  "submissionId": "string",
  "score": "number",
  "feedback": "string",
  "completed": "boolean",
  "nextModuleId": "string"
}
```

**Response (400 Bad Request)**:
```json
{
  "error": "string",
  "details": "object"
}
```

### Get Assessment Feedback
- **Endpoint**: `GET /api/assessments/:assessmentId/feedback`
- **Description**: Retrieve feedback for a submitted assessment
- **Authentication**: Required (JWT token)

**Response (200 OK)**:
```json
{
  "assessmentId": "string",
  "userId": "string",
  "score": "number",
  "maxScore": "number",
  "feedback": [
    {
      "questionId": "string",
      "questionText": "string",
      "userAnswer": "string",
      "correctAnswer": "string",
      "isCorrect": "boolean",
      "explanation": "string"
    }
  ],
  "overallFeedback": "string"
}
```

## Progress Tracking API

### Save User Progress
- **Endpoint**: `POST /api/progress/save`
- **Description**: Save a user's progress through the curriculum
- **Authentication**: Required (JWT token)

**Request Body**:
```json
{
  "userId": "string",
  "moduleId": "string",
  "weekNumber": "number",
  "completedSections": ["string"],
  "completionPercentage": "number",
  "lastAccessed": "ISO 8601 datetime",
  "timeSpent": "number" // in minutes
}
```

**Response (200 OK)**:
```json
{
  "progressId": "string",
  "savedAt": "ISO 8601 datetime",
  "status": "string"
}
```

### Get User Progress
- **Endpoint**: `GET /api/progress/:userId`
- **Description**: Retrieve a user's progress across the curriculum
- **Authentication**: Required (JWT token)

**Response (200 OK)**:
```json
{
  "userId": "string",
  "overallCompletion": "number", // percentage
  "modules": [
    {
      "moduleId": "string",
      "title": "string",
      "completionPercentage": "number",
      "weeks": [
        {
          "weekNumber": "number",
          "completed": "boolean",
          "completedAt": "ISO 8601 datetime"
        }
      ],
      "completedAt": "ISO 8601 datetime",
      "timeSpent": "number" // in minutes
    }
  ],
  "lastAccessedModule": "string",
  "estimatedCompletionDays": "number"
}
```

## Simulation Interaction API

### Run Simulation
- **Endpoint**: `POST /api/simulation/run`
- **Description**: Execute a robotics simulation and return results
- **Authentication**: Required (JWT token)

**Request Body**:
```json
{
  "userId": "string",
  "simulationId": "string",
  "parameters": "object", // simulation-specific parameters
  "duration": "number" // in seconds
}
```

**Response (200 OK)**:
```json
{
  "simulationId": "string",
  "executionId": "string",
  "status": "string", // "running", "completed", "failed"
  "results": "object", // simulation-specific results
  "timestamp": "ISO 8601 datetime"
}
```

### Get Simulation Results
- **Endpoint**: `GET /api/simulation/results/:executionId`
- **Description**: Retrieve results from a completed simulation
- **Authentication**: Required (JWT token)

**Response (200 OK)**:
```json
{
  "executionId": "string",
  "simulationId": "string",
  "status": "string", // "completed", "failed"
  "results": "object", // simulation-specific results
  "metrics": {
    "successRate": "number",
    "efficiency": "number",
    "accuracy": "number"
  },
  "timestamp": "ISO 8601 datetime"
}
```

## Error Response Format

All error responses follow this format:

```json
{
  "error": {
    "code": "string", // e.g., "VALIDATION_ERROR", "NOT_FOUND", "UNAUTHORIZED"
    "message": "string", // human-readable error message
    "details": "object" // optional details about the error
  }
}
```

## Common Headers

- `Authorization: Bearer <JWT_TOKEN>` - Required for authenticated endpoints
- `Content-Type: application/json` - For requests with JSON body
- `Accept: application/json` - For responses in JSON format