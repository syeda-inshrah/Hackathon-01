# API Contract: Task State Machine (Behavior Tree) Interface

**Interface**: TaskStateMachine
**Service**: Module 4 - Integrated Humanoid Task Execution
**Version**: 1.0

## Purpose

This contract defines the API for the Behavior Tree-based task state machine that manages the complete fetch-and-place workflow. The state machine orchestrates perception, navigation, and manipulation components through the entire task execution lifecycle.

## Behavior Tree Structure

The Behavior Tree for the fetch-and-place task follows this structure:

```
Root
├── Sequence: FetchAndPlaceTask
    ├── Selector: NavigateToTableOrAbort
    │   ├── Sequence: NavigateToTable
    │   │   ├── Condition: PathToTableClear
    │   │   ├── Action: MoveToTableLocation
    │   │   └── Condition: AtTableLocation
    │   └── Action: AbortTask
    ├── Selector: LocalizeObjectOrRetry
    │   ├── Sequence: ObjectLocalizeSuccess
    │   │   ├── Action: DetectObject
    │   │   └── Condition: ObjectDetectedWithConfidence
    │   └── Action: RetryLocalization
    ├── Selector: GraspObjectOrFail
    │   ├── Sequence: GraspSuccess
    │   │   ├── Action: ApproachObject
    │   │   ├── Action: ExecuteGrasp
    │   │   └── Condition: ObjectGraspedSuccessfully
    │   └── Action: GraspFailedAction
    ├── Selector: TransportToObjectMatOrStabilize
    │   ├── Sequence: TransportSuccess
    │   │   ├── Action: NavigateToMat
    │   │   └── Condition: AtMatLocation
    │   └── Action: StabilizeAndContinue
    ├── Selector: PlaceObjectOrRetry
    │   ├── Sequence: PlaceSuccess
    │   │   ├── Action: PositionForPlacement
    │   │   ├── Action: ExecutePlacement
    │   │   └── Condition: ObjectPlacedSuccessfully
    │   └── Action: RetryPlacement
    └── Condition: TaskCompletedSuccessfully
```

## Action Definitions

### Action: ExecuteBehaviorTree
Executes a complete behavior tree with specified root goal.

**Goal Message**:
```json
{
  "tree_definition": {
    "root_node": "string (identifier for the root node of the behavior tree)",
    "nodes": [
      {
        "id": "string (unique node identifier)",
        "type": "string ('action', 'condition', 'sequence', 'selector', 'parallel', 'decorator')",
        "name": "string (descriptive name)",
        "children": ["string (list of child node IDs)"],
        "parameters": "dict (node-specific parameters)",
        "decorators": ["string (list of decorators like 'inverter', 'repeat_until_success')"]  
      }
    ],
    "blackboard_vars": {
      "task_destination": {"x": "float", "y": "float", "z": "float"},
      "target_object": "string (identifier for the object to manipulate)",
      "object_pose": {"position": {}, "orientation": {}},
      "robot_pose": {"position": {}, "orientation": {}},
      "current_phase": "string ('navigating_to_table', 'localizing_object', etc.)",
      "execution_status": "string ('running', 'success', 'failure', 'canceled')"
    }
  },
  "execution_params": {
    "timeout": "float (maximum execution time in seconds)",
    "failure_recovery_enabled": "boolean",
    "verbose_logging": "boolean",
    "restart_on_failure": "int (number of times to restart on failure)"
  }
}
```

**Result Message**:
```json
{
  "execution_id": "string (unique identifier for this execution)",
  "success": "boolean",
  "final_status": "string ('SUCCESS', 'FAILURE', 'CANCELED', 'TIMEOUT')",
  "execution_time": "float (time taken in seconds)",
  "visited_nodes": ["string (list of node IDs that were visited)"],
  "failed_nodes": ["string (list of nodes that failed)"],
  "recovery_attempts": "int (number of recovery attempts made)",
  "final_blackboard_state": "dict (final values of all blackboard variables)",
  "performance_metrics": {
    "average_node_execution_time": "float (in ms)",
    "max_node_execution_time": "float (in ms)",
    "total_decision_points": "int",
    "success_rate": "float (percentage)"
  },
  "error_message": "string (if applicable)"
}
```

**Feedback Message**:
```json
{
  "current_node_id": "string (currently executing node)",
  "current_node_status": "string ('RUNNING', 'SUCCESS', 'FAILURE')",
  "current_blackboard_state": "dict (current values of relevant blackboard variables)",
  "path_to_leaf": ["string (path from root to current node)"],
  "estimated_time_remaining": "float (in seconds)",
  "execution_trace": [
    {
      "node_id": "string",
      "status": "string",
      "timestamp": "string (ISO 8601)",
      "duration_ms": "float"
    }
  ],
  "last_event": "string (last significant event in execution)",
  "recovery_active": "boolean (whether a recovery procedure is currently active)"
}
```

---

### Action: NavigateToTable
Navigates the humanoid robot to the table location where the target object is located.

**Goal Message**:
```json
{
  "table_location": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "approach_vector": {
    "x": "float (direction to approach the table)",
    "y": "float",
    "z": "float"
  },
  "safety_margin": "float (minimum distance to maintain from table edges)",
  "approach_height": "float (desired height to approach the table in meters)",
  "locomotion_params": {
    "step_size": "float (default step size in meters)",
    "walking_speed": "float (walking velocity in m/s)",
    "foot_lift_height": "float (height to lift feet during approach in meters)",
    "balance_preservation": "float (priority for balance preservation vs speed, 0.0-1.0)"
  },
  "obstacle_avoidance": {
    "enabled": "boolean",
    "detection_range": "float (range in meters to detect obstacles)",
    "planning_horizon": "float (distance ahead for path planning in meters)"
  }
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "final_robot_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "distance_to_target": "float (final distance to target position in meters)",
  "approach_success": "boolean",
  "navigation_metrics": {
    "path_length": "float (actual path length traveled in meters)",
    "execution_time": "float (time taken in seconds)",
    "steps_taken": "int (total number of steps)",
    "balance_maintained_percentage": "float (percentage of time balance was maintained)",
    "obstacle_avoided_count": "int (number of obstacles avoided during navigation)"
  },
  "error_message": "string (if success is false)"
}
```

---

### Action: LocalizeObject
Localizes the target object for manipulation.

**Goal Message**:
```json
{
  "object_descriptor": {
    "type": "string (object type: 'cube', 'cylinder', 'sphere', etc.)",
    "color": "string (optional color specification)",
    "size_range": {
      "min": {"x": "float", "y": "float", "z": "float"},
      "max": {"x": "float", "y": "float", "z": "float"}
    },
    "location_hint": {
      "position": {"x": "float", "y": "float", "z": "float"},
      "search_radius": "float (radius to search around hint position in meters)"
    }
  },
  "sensor_config": {
    "use_depth_camera": "boolean",
    "use_lidar": "boolean",
    "confidence_threshold": "float (minimum detection confidence, 0.0-1.0)",
    "max_detections": "int (maximum number of objects to consider)"
  },
  "coordinate_frame": "string (coordinate frame for returning object pose)",
  "refinement_params": {
    "refinement_enabled": "boolean",
    "refinement_iterations": "int",
    "accuracy_threshold": "float (desired accuracy in meters)"
  }
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "object_found": "boolean",
  "object_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
    "covariance": ["float (diagonal elements of 6x6 pose covariance matrix)"]
  },
  "confidence": "float (detection confidence, 0.0-1.0)",
  "detection_time": "float (time taken for localization in seconds)",
  "sensor_fusion_used": "boolean (whether multiple sensors were used)",
  "localization_metrics": {
    "accuracy": "float (localization accuracy in meters)",
    "computation_time": "float (time for processing in seconds)",
    "quality_score": "float (overall quality of detection, 0.0-1.0)"
  },
  "error_message": "string (if success is false)"
}
```

---

### Action: ExecuteGrasp
Executes the grasp operation to pick up the localized object.

**Goal Message**:
```json
{
  "object_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "grasp_strategy": "string ('top_down', 'side_grasp', 'pinch', 'wrap', 'suction')",
  "pre_grasp_offset": {
    "x": "float (offset before grasp in meters)",
    "y": "float",
    "z": "float"
  },
  "grasp_approach": {
    "direction": {"x": "float", "y": "float", "z": "float"},
    "distance": "float (distance to approach in meters)"
  },
  "grasp_params": {
    "gripper_width": "float (desired gripper width in meters)",
    "grasp_force": "float (grasp force in Newtons)",
    "grasp_velocity": "float (velocity during grasp in m/s)",
    "attempt_backup": "boolean (whether to try backup grasp points if primary fails)",
    "force_tolerance": "float (force threshold for successful grasp in Newtons)"
  },
  "balance_params": {
    "support_foot": "string ('left', 'right', 'both')",
    "crouch_during_grasp": "boolean",
    "weight_shift_distance": "float (distance to shift weight in meters)"
  }
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "grasp_successful": "boolean",
  "object_attached": "boolean",
  "grasp_quality_score": "float (quality score of the grasp, 0.0-1.0)",
  "estimated_object_weight": "float (estimated weight of object in kg)",
  "manipulation_metrics": {
    "approach_time": "float (time to approach object in seconds)",
    "grasp_time": "float (time to execute grasp in seconds)",
    "lift_time": "float (time to lift object in seconds)",
    "balance_preserved_percentage": "float (percentage of time balance was maintained)",
    "force_profile": {"min": "float", "max": "float", "avg": "float"}
  },
  "error_message": "string (if success is false)",
  "recovery_suggestions": ["string (list of possible recovery strategies)"]
}
```

---

### Action: NavigateToMat
Navigates the humanoid robot to the mat location while carrying the object.

**Goal Message**:
```json
{
  "destination": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "carrying_mode": "string ('high_transport', 'low_transport', 'cradle')",
  "object_safety_params": {
    "min_clearance": "float (minimum clearance from obstacles while carrying in meters)",
    "max_tilt_angle": "float (maximum robot tilt while carrying in radians)",
    "collision_risk_threshold": "float (risk threshold for carrying in 0.0-1.0)"
  },
  "transport_specific_params": {
    "arm_configuration": "string ('fixed', 'adaptive') (how to configure arm during transport)",
    "gait_adaptation": "string ('normal', 'careful', 'stable') (walking pattern for transport)",
    "balance_priority_weight": "float (priority for balance vs speed during transport, 0.0-1.0)"
  }
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "object_still_attached": "boolean",
  "final_destination_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "transport_stability": "float (measure of stability during transport, 0.0-1.0)",
  "navigation_metrics": {
    "path_length": "float (actual path length traveled in meters)",
    "execution_time": "float (time taken in seconds)",
    "steps_taken": "int (total number of steps)",
    "balance_maintained_percentage": "float (percentage of time balance was maintained)",
    "obj_dropped_count": "int (number of times object almost dropped)"
  },
  "error_message": "string (if success is false)"
}
```

---

### Service: GetTaskStatus
Retrieves the current status of a behavior tree execution.

**Request**:
```json
{
  "execution_id": "string (ID of the behavior tree execution to query)"
}
```

**Response**:
```json
{
  "execution_id": "string",
  "current_status": "string ('RUNNING', 'SUCCESS', 'FAILURE', 'CANCELED', 'PAUSED')",
  "current_node": "string (ID of currently active node)",
  "current_phase": "string ('navigation', 'localization', 'manipulation', 'transport', etc.)",
  "execution_time": "float (time since execution started in seconds)",
  "estimated_remaining_time": "float (estimated time remaining in seconds)",
  "success_ratio": "float (percentage of completed nodes that succeeded)",
  "blackboard_snapshot": "dict (current state of all blackboard variables)",
  "recent_events": [
    {
      "timestamp": "string",
      "event_type": "string",
      "node_id": "string",
      "details": "string"
    }
  ],
  "performance_metrics": {
    "average_node_processing_time": "float",
    "max_concurrent_nodes": "int",
    "memory_usage_mb": "float"
  }
}
```

**Errors**:
- ExecutionNotFound: If the specified execution ID does not exist
- InvalidExecutionId: If the execution ID is malformed