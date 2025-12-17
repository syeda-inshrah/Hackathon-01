# API Contract: Task Orchestrator Interface

**Interface**: TaskOrchestrator
**Service**: Module 4 - Integrated Humanoid Task Execution
**Version**: 1.0

## Purpose

This contract defines the API interface for the central task orchestrator that coordinates the fetch-and-place task between all modules. The orchestrator manages state transitions and coordinates perception, navigation, and manipulation components.

## Endpoints

### POST /tasks/fetch_and_place
Initiates a complete fetch-and-place task with specified object and destination.

**Request Body**:
```json
{
  "task_id": "string (unique identifier for this task)",
  "object_descriptor": {
    "type": "string (type of object to fetch, e.g., 'cube', 'cylinder')",
    "color": "string (optional color to help identify the object)",
    "position_hint": {
      "x": "float (approximate x position of the object)",
      "y": "float (approximate y position of the object)",
      "z": "float (approximate z position of the object)"
    },
    "size": {
      "x": "float (approximate width in meters)",
      "y": "float (approximate depth in meters)",
      "z": "float (approximate height in meters)"
    }
  },
  "destination": {
    "position": {
      "x": "float (x coordinate of destination)",
      "y": "float (y coordinate of destination)",
      "z": "float (z coordinate of destination)"
    },
    "orientation": {
      "x": "float (x component of quaternion)",
      "y": "float (y component of quaternion)",
      "z": "float (z component of quaternion)",
      "w": "float (w component of quaternion)"
    }
  },
  "options": {
    "timeout": "float (max time for task completion in seconds)",
    "retries": "int (number of retry attempts if grasp fails)",
    "approach_height": "float (height to approach object from, in meters)",
    "grasp_strategy": "string ('precise', 'power', 'pinch', etc.)"
  }
}
```

**Response**:
```json
{
  "success": "boolean (true if task initiation was successful)",
  "task_handle": "string (identifier to track the task progress)",
  "estimated_duration": "float (estimated time for task completion in seconds)",
  "next_state": "string (expected next state of the task)",
  "error": "string (if success is false, description of the error)"
}
```

**Errors**:
- InvalidObjectDescriptorError: If the object descriptor is malformed or invalid
- DestinationUnreachableError: If the destination is not reachable by the robot
- RobotBusyError: If the robot is already executing another task
- PerceptionNotReadyError: If perception system is not responding

---

### GET /tasks/{task_id}/status
Retrieves the current status of a fetch-and-place task.

**Response**:
```json
{
  "task_id": "string",
  "status": "string ('idle', 'navigating_to_table', 'localizing_object', 'grasping', 'transporting', 'navigating_to_mat', 'placing', 'completed', 'failed')",
  "current_phase": "string",
  "elapsed_time": "float (seconds since task started)",
  "estimated_remaining_time": "float (estimated seconds remaining)",
  "robot_state": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
    "gripper_state": "string ('open', 'closed', 'partially_closed')"
  },
  "object_state": {
    "detected": "boolean",
    "position": {"x": "float", "y": "float", "z": "float"},
    "relative_position": {"x": "float", "y": "float", "z": "float"}
  },
  "last_error": "string (if any error occurred recently)",
  "progress_percentage": "float (0-100 percentage of task completion)"
}
```

**Errors**:
- TaskNotFoundError: If the task_id does not exist or has expired
- CommunicationError: If unable to retrieve current status from orchestrator

---

### POST /tasks/{task_id}/cancel
Cancels a currently executing fetch-and-place task.

**Response**:
```json
{
  "success": "boolean",
  "task_id": "string",
  "previous_state": "string",
  "final_state": "string",
  "error_count": "int (number of errors encountered before cancellation)",
  "cancellation_reason": "string (why the task was cancelled)"
}
```

**Errors**:
- TaskNotFoundError: If the task_id does not exist
- InvalidTaskStateError: If the task is already completed and cannot be cancelled

---

### PUT /tasks/{task_id}/update_destination
Updates the destination for an ongoing fetch-and-place task.

**Request Body**:
```json
{
  "destination": {
    "position": {
      "x": "float (new x coordinate of destination)",
      "y": "float (new y coordinate of destination)",
      "z": "float (new z coordinate of destination)"
    },
    "orientation": {
      "x": "float (new x component of quaternion)",
      "y": "float (new y component of quaternion)",
      "z": "float (new z component of quaternion)",
      "w": "float (new w component of quaternion)"
    }
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "task_id": "string",
  "previous_destination": "object (the previous destination pose)",
  "new_destination": "object (the updated destination pose)",
  "message": "string (confirmation message)"
}
```

**Errors**:
- TaskNotFoundError: If the task_id does not exist
- InvalidDestinationError: If the new destination is malformed
- InvalidTaskStateError: If the task cannot be modified in its current phase
- DestinationUnreachableError: If the new destination is not reachable