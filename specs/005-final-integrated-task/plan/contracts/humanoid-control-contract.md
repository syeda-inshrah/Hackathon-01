# API Contract: Humanoid Control Interface (Whole-Body Control)

**Interface**: HumanoidControlInterface
**Service**: Module 4 - Integrated Humanoid Task Execution
**Version**: 1.0

## Purpose

This contract defines the ROS 2 Action interface for high-level humanoid control commands, including navigation, manipulation, and coordinated locomotion-manipulation tasks. It assumes a low-level Whole-Body Controller (WBC) exists and provides a high-level abstraction for task execution.

## Action Definitions

### Action: WalkToGoal
Moves the humanoid robot to a specified location using bipedal locomotion.

**Goal Message**:
```json
{
  "target_pose": {
    "position": {
      "x": "float (target x coordinate in meters)",
      "y": "float (target y coordinate in meters)", 
      "z": "float (target z coordinate in meters)"
    },
    "orientation": {
      "x": "float (target x orientation in quaternion)",
      "y": "float (target y orientation in quaternion)",
      "z": "float (target z orientation in quaternion)",
      "w": "float (target w orientation in quaternion)"
    }
  },
  "tolerance": {
    "position": "float (position tolerance in meters)",
    "orientation": "float (orientation tolerance in radians)"
  },
  "locomotion_style": "string ('natural_walk', 'fast_walk', 'careful_walk', 'stealth_walk')",
  "stability_margin": "float (minimum stability margin for footstep planning in meters)",
  "max_step_size": "float (maximum allowable step size in meters)",
  "foot_lift_height": "float (height to lift feet during walking in meters)"
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "final_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "path_executed": [
    {
      "position": {"x": "float", "y": "float", "z": "float"},
      "timestamp": "float (time when robot reached this position)"
    }
  ],
  "metrics": {
    "execution_time": "float (seconds to complete navigation)",
    "steps_taken": "int (total number of steps)",
    "balance_preserved": "float (percentage of time balance was maintained during navigation)",
    "deviation_from_path": "float (average deviation from planned path in meters)"
  },
  "error_message": "string (if success is false)"
}
```

**Feedback Message**:
```json
{
  "current_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "footstep_plan": [
    {"x": "float", "y": "float", "z": "float", "step_number": "int"}
  ],
  "current_step": "int (current step in the plan)",
  "total_steps": "int (total steps in the plan)",
  "balance_margin": "float (current balance stability margin)",
  "estimated_time_remaining": "float (estimated seconds remaining for navigation)"
}
```

---

### Action: PickUpObject
Controls the humanoid robot to approach, grasp, and pick up an object.

**Goal Message**:
```json
{
  "object_id": "string (identifier of the object to pick up)",
  "object_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "approach_direction": {
    "x": "float (direction vector for approach)",
    "y": "float",
    "z": "float"
  },
  "grasp_type": "string ('top_down', 'side_grasp', 'pinch', 'power', 'suction')",
  "grasp_pose": {
    "position_offset": {"x": "float", "y": "float", "z": "float"},
    "orientation_offset": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "gripper_params": {
    "force_limit": "float (maximum gripping force in Newtons)",
    "velocity": "float (gripper closing velocity in m/s)"
  },
  "support_foot": "string ('left', 'right', 'both') (which foot to use for balance during grasp)",
  "backup_plan": "string ('retry_grasp', 'move_and_retry', 'abort_task') (what to do if grasp fails)"
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "grasp_confidence": "float (confidence in successful grasp, 0.0-1.0)",
  "object_attached": "boolean (whether object is believed to be attached to end effector)",
  "final_manipulator_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "manipulation_metrics": {
    "approach_success": "boolean (successfully approached object)",
    "grasp_success": "boolean (successfully grasped object)",
    "lift_success": "boolean (successfully lifted object)",
    "balance_preserved": "float (percentage of time balance was maintained during manipulation)",
    "execution_time": "float (seconds to complete manipulation)"
  },
  "error_message": "string (if success is false)",
  "recovery_attempts": "int (number of recovery attempts made)"
}
```

**Feedback Message**:
```json
{
  "current_phase": "string ('approaching', 'positioning', 'grasping', 'lifting', 'completed')",
  "manipulator_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "end_effector_state": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
    "gripper_opening": "float (gripper opening in meters)"
  },
  "balance_metrics": {
    "center_of_mass": {"x": "float", "y": "float", "z": "float"},
    "support_polygon_center": {"x": "float", "y": "float"},
    "stability_margin": "float (current stability margin in meters)"
  },
  "estimated_time_remaining": "float (seconds remaining to complete manipulation)"
}
```

---

### Action: PlaceObject
Controls the humanoid robot to place an object at a specified location.

**Goal Message**:
```json
{
  "placement_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "placement_surface": "string ('table', 'mat', 'floor', 'shelf')",
  "approach_direction": {
    "x": "float (direction vector for approach)",
    "y": "float",
    "z": "float"
  },
  "release_params": {
    "force_release": "boolean (whether to release with specified force)",
    "velocity": "float (gripper opening velocity in m/s)",
    "settle_time": "float (time to wait after placement in seconds)"
  },
  "stability_check": "boolean (whether to verify placement stability before releasing)",
  "support_foot": "string ('left', 'right', 'both') (which foot to use for balance during placement)",
  "backup_plan": "string ('retry_placement', 'adjust_and_retry', 'abort_task') (what to do if placement fails)"
}
```

**Result Message**:
```json
{
  "success": "boolean",
  "placement_accuracy": "float (accuracy of placement relative to target in meters)",
  "object_released": "boolean (whether object is confirmed to be released)",
  "final_placement_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "placement_metrics": {
    "approach_success": "boolean",
    "positioning_success": "boolean",
    "release_success": "boolean",
    "stability_after_placement": "float (stability metric after object release)",
    "execution_time": "float (seconds to complete placement)"
  },
  "error_message": "string (if success is false)", 
  "verification_result": {
    "visual_confirmation": "boolean (confirmation via visual feedback)",
    "force_confirmation": "boolean (confirmation via force/torque sensors)",
    "position_verification": "boolean (confirmation of position accuracy)"
  }
}
```

**Feedback Message**:
```json
{
  "current_phase": "string ('approaching', 'positioning', 'releasing', 'verifying', 'completed')",
  "manipulator_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "end_effector_state": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
    "gripper_opening": "float (gripper opening in meters)",
    "holding_object": "boolean (whether object is currently held)"
  },
  "balance_metrics": {
    "center_of_mass": {"x": "float", "y": "float", "z": "float"},
    "support_polygon_center": {"x": "float", "y": "float"},
    "stability_margin": "float (current stability margin in meters)"
  },
  "estimated_time_remaining": "float (seconds remaining to complete placement)"
}
```

---

## Service Endpoints

### POST /control/emergency_stop
Immediately stops all ongoing humanoid control actions for safety.

**Request Body**:
```json
{
  "emergency_reason": "string ('user_request', 'balance_loss', 'obstacle_detected', 'hardware_fault', 'communication_error')",
  "safe_posture": "string ('stance', 'crouch', 'shutdown') (how to position robot after stopping)",
  "log_incident": "boolean (whether to create an incident log entry)"
}
```

**Response**:
```json
{
  "success": "boolean",
  "actions_stopped": ["string (list of stopped action IDs)"],
  "final_robot_state": {
    "balance_preserved": "boolean",
    "position": {"x": "float", "y": "float", "z": "float"},
    "joint_positions": {"joint_name": "float (position in radians/degrees)"}
  },
  "incident_logged": "boolean",
  "message": "string (confirmation message)"
}
```

**Errors**:
- SystemAlreadyStoppedError: If the system is already in a safe stop state
- InvalidSafePostureError: If the requested safe posture is not achievable in current situation