# Nav2 Humanoid Adaption API Contract

**Interface**: BipedalNav2Interface
**Implementation**: C++/Python with ROS 2 Navigation2 stack
**Version**: 1.0

## Purpose

This contract defines the API for adapting the Navigation2 stack for bipedal humanoid robots, including specialized planners that account for stability margins and kinematic constraints of bipedal locomotion.

## Endpoints

### POST /nav2/bipedal/configure
Configure Navigation2 for bipedal humanoid navigation with specialized stability considerations.

**Request Body**:
```json
{
  "robot_model": "string (URDF/SDF path for the humanoid robot)",
  "foot_separation": "float (distance between feet in meters)",
  "stability_margin": "float (minimum stability margin in meters)",
  "step_size_limits": {
    "max_forward": "float (max forward step size)",
    "max_backward": "float (max backward step size)", 
    "max_lateral": "float (max lateral step size)",
    "max_rotation": "float (max rotational step in radians)"
  },
  "planning_constraints": {
    "max_com_height_variation": "float (max allowable CoM height change)",
    "support_polygon_tolerance": "float (tolerance for support polygon)",
    "zero_moment_point_limits": {
      "x_range": {"min": "float", "max": "float"},
      "y_range": {"min": "float", "max": "float"}
    }
  },
  "local_planner": "string (name of the local planner to use)",
  "global_planner": "string (name of the global planner to use)"
}
```

**Response**:
```json
{
  "success": "boolean",
  "configured_params": "dict (parameters that were successfully configured)",
  "warnings": ["string (any configuration warnings)"],
  "message": "string"
}
```

**Errors**:
- InvalidRobotModelError: If the robot model doesn't represent a bipedal humanoid
- StabilityConstraintsViolationError: If stability constraints are invalid
- UnsupportedPlannerError: If specified planners don't support bipedal modifications

### POST /nav2/bipedal/goto_pose
Send a navigation goal for bipedal humanoid navigation considering stability constraints.

**Request Body**:
```json
{
  "target_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "planning_options": {
    "consider_support_polygons": "boolean",
    "maintain_stability_margin": "boolean",
    "preferred_foot_step_pattern": "string (\"alternating\", \"custom\")",
    "stability_threshold": "float (minimum stability value required)",
    "motion_primitive_set": "string (which motion primitives to consider)"
  },
  "timeout": "float (timeout for navigation in seconds)"
}
```

**Response**:
```json
{
  "success": "boolean",
  "navigation_task_id": "string",
  "initial_support_state": {
    "left_foot": {"x": "float", "y": "float", "z": "float"},
    "right_foot": {"x": "float", "y": "float", "z": "float"},
    "center_of_mass": {"x": "float", "y": "float", "z": "float"}
  },
  "estimated_duration": "float (estimated time to completion in seconds)",
  "status": "string (\"planning\", \"executing\", \"failed\")"
}
```

**Errors**:
- GoalUnreachableError: If the goal is unreachable with stability constraints
- InvalidPoseError: If the target pose is invalid
- PlanningTimeoutError: If planning takes longer than allowed

### GET /nav2/bipedal/status/{task_id}
Get the current status of a bipedal navigation task.

**Response**:
```json
{
  "task_id": "string",
  "status": "string (\"planning\", \"executing\", \"completed\", \"cancelled\", \"failed\")",
  "current_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "current_support_state": {
    "left_foot": {"x": "float", "y": "float", "z": "float"},
    "right_foot": {"x": "float", "y": "float", "z": "float"},
    "center_of_mass": {"x": "float", "y": "float", "z": "float"},
    "zmp_position": {"x": "float", "y": "float"}
  },
  "stability_metric": "float (current stability value)",
  "progress_percent": "float",
  "estimated_remaining_time": "float"
}
```

**Errors**:
- TaskNotFoundError: If the navigation task doesn't exist

### PUT /nav2/bipedal/update_constraints
Update the stability and kinematic constraints for the bipedal navigator.

**Request Body**:
```json
{
  "new_stability_margin": "float (new minimum stability margin in meters)",
  "new_step_size_limits": {
    "max_forward": "float (new max forward step size)",
    "max_backward": "float (new max backward step size)",
    "max_lateral": "float (new max lateral step size)",
    "max_rotation": "float (new max rotation step in radians)"
  },
  "new_com_height_limits": {
    "min_height": "float (minimum CoM height)",
    "max_height": "float (maximum CoM height)"
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "updated_params": "dict (parameters that were updated)",
  "message": "string"
}
```

**Errors**:
- InvalidConstraintsError: If new constraints are invalid or unsafe
- ActiveNavigationError: If an active navigation is happening when parameters are changed

### POST /nav2/bipedal/cancel/{task_id}
Cancel an active bipedal navigation task.

**Response**:
```json
{
  "success": "boolean",
  "cancelled_task_id": "string",
  "final_support_state": {
    "left_foot": {"x": "float", "y": "float", "z": "float"},
    "right_foot": {"x": "float", "y": "float", "z": "float"},
    "center_of_mass": {"x": "float", "y": "float", "z": "float"}
  },
  "message": "string"
}
```

**Errors**:
- TaskNotFoundError: If the task doesn't exist
- AlreadyFinishedError: If the task has already completed