# Data Model: Module 4 - Integrated Humanoid Task Execution

**Feature**: Module 4: Integrated Humanoid Task Execution
**Branch**: 005-final-integrated-task
**Created**: 2025-12-15
**Status**: Complete

## Entities

### FetchAndPlaceTask
Represents the complete fetch-and-place task workflow

**Fields**:
- task_id: string (unique identifier for this task instance)
- status: string (current status: "idle", "navigating_to_table", "localizing_object", "grasping", "transporting", "navigating_to_mat", "placing", "completed", "failed")
- start_time: datetime (when the task was initiated)
- end_time: datetime (when the task was completed or failed)
- target_object: ObjectInfo (information about the object to fetch)
- destination: Pose (location where object should be placed)
- robot_state_history: list of RobotState (tracking states throughout task execution)
- error_log: list of ErrorEvent (errors encountered during execution)

**Relationships**:
- One-to-one with ObjectInfo (one target object per task)
- One-to-many with RobotState (multiple robot states during task)
- One-to-many with ErrorEvent (multiple errors possible during task)
- One-to-one with TaskOutcome (result of the task execution)

**Validation rules**:
- task_id must be unique within the system
- status must be one of the defined values
- target_object must be valid and accessible
- destination must be reachable by the robot

### ObjectInfo
Information about objects in the environment relevant to the task

**Fields**:
- object_id: string (unique identifier for the object)
- class_name: string (classification of the object: "cube", "cylinder", etc.)
- pose: Pose (current 3D position and orientation of the object)
- dimensions: Vector3 (size of the object in x, y, z dimensions)
- mass: float (weight of the object in kg)
- grasp_points: list of GraspPoint (possible grasp locations on the object)
- confidence: float (confidence value of object detection in range 0-1)

**Relationships**:
- Many-to-one with FetchAndPlaceTask (multiple objects, one target per task)
- One-to-many with GraspPoint (one object has many possible grasp points)

**Validation rules**:
- object_id must be unique
- pose must have valid coordinates
- dimensions must be positive values
- confidence must be between 0 and 1

### RobotState
Represents the current state of the robot during task execution

**Fields**:
- timestamp: datetime (when this state was captured)
- base_pose: Pose (position and orientation of robot base)
- joint_positions: dict (mapping of joint names to position values)
- joint_velocities: dict (mapping of joint names to velocity values)
- joint_efforts: dict (mapping of joint names to effort values)
- end_effector_pose: Pose (pose of the end effector/grasping mechanism)
- gripper_state: string ("open", "closed", "partially_closed")
- center_of_mass: Vector3 (current center of mass of the robot)
- support_polygon: list of Point (vertices of the robot's current support polygon)

**Relationships**:
- Many-to-one with FetchAndPlaceTask (multiple states during one task)
- Many-to-one with BasePose (current pose of robot base)
- Many-to-one with EndEffectorPose (current pose of end effector)

**Validation rules**:
- timestamp must be current or recent
- joint_positions values must be within joint limits
- gripper_state must be one of the defined values

### GraspPoint
Represents a possible location on an object where it can be grasped

**Fields**:
- position: Point (position relative to object origin where grasp can occur)
- orientation: Quaternion (recommended orientation for approaching the grasp point)
- approach_direction: Vector3 (direction to approach for successful grasp)
- grasp_type: string ("top_down", "side_grasp", "pinch", "power", etc.)
- stability_score: float (score representing how stable this grasp would be)
- object_ref: ObjectInfo (reference to the object this grasp point belongs to)

**Relationships**:
- Many-to-one with ObjectInfo (multiple grasp points per object)
- One-to-many with GraspAttempt (one grasp point may have multiple attempted grasps)

**Validation rules**:
- position must be within the object's bounds
- stability_score must be between 0 and 1
- grasp_type must be one of the recognized types

### TaskOutcome
Result and metrics of a completed task

**Fields**:
- success: boolean (whether the task was completed successfully)
- completion_time: float (total time to complete the task in seconds)
- success_rate: float (percentage of task completed successfully)
- error_count: int (number of errors encountered during task)
- final_state: RobotState (state of the robot at task completion)
- verification_result: VerificationResult (result of post-task verification)
- performance_metrics: PerformanceMetrics (detailed metrics collected during task)

**Relationships**:
- One-to-one with FetchAndPlaceTask (one outcome per task)
- One-to-one with VerificationResult (one verification result per task)
- One-to-one with PerformanceMetrics (one performance summary per task)

**Validation rules**:
- success must be a boolean value
- completion_time must be positive if task completed
- success_rate must be between 0 and 100

### ErrorEvent
Represents an error or exception encountered during task execution

**Fields**:
- timestamp: datetime (when the error occurred)
- error_type: string (type of error: "navigation_failure", "grasp_failure", "communication_error", etc.)
- description: string (detailed description of the error)
- severity: string ("low", "medium", "high", "critical")
- recovery_attempted: boolean (whether recovery was attempted)
- recovery_success: boolean (whether recovery was successful)
- task_phase: string (phase of task when error occurred)
- error_code: string (machine-readable error code)

**Relationships**:
- Many-to-one with FetchAndPlaceTask (multiple errors per task)
- One-to-many with RecoveryAttempt (one error may have multiple recovery attempts)

**Validation rules**:
- timestamp must be valid
- error_type must be one of defined error types
- severity must be one of defined severity levels

### BehaviorTreeNode
Represents a node in the Behavior Tree managing the task

**Fields**:
- node_id: string (unique identifier for the node)
- node_type: string ("action", "condition", "sequence", "selector", "decorator")
- name: string (descriptive name for the node)
- status: string ("idle", "running", "success", "failure", "aborted")
- children: list of BehaviorTreeNode (child nodes for composite nodes)
- parameters: dict (parameters required for this node)
- parent: BehaviorTreeNode (parent node, null for root)
- execution_order: int (order in which this node should execute among siblings)

**Relationships**:
- One-to-many with BehaviorTreeNode (one parent has many children)
- Many-to-one with BehaviorTreeNode (many children have one parent)
- One-to-one with TaskPhase (each node maps to a task phase)

**Validation rules**:
- node_id must be unique within the tree
- node_type must be one of the valid Behavior Tree node types
- status transitions must follow valid patterns
- execution_order must be non-negative

### VerificationResult
Represents the result of verifying task completion

**Fields**:
- verification_id: string (unique identifier for this verification)
- object_placed: boolean (whether object is confirmed to be at destination)
- placement_accuracy: float (distance between object and target destination in meters)
- grasp_success: boolean (whether object was successfully grasped)
- transport_success: boolean (whether object was transported without dropping)
- confidence_score: float (overall confidence in task completion in range 0-1)
- verification_details: dict (detailed results for each verification step)

**Relationships**:
- One-to-one with TaskOutcome (one verification per outcome)
- One-to-many with VerificationDetail (one verification has multiple details)

**Validation rules**:
- confidence_score must be between 0 and 1
- verification_id must be unique

### PerformanceMetrics
Collection of performance metrics during task execution

**Fields**:
- avg_computation_time: float (average time for processing each step in seconds)
- max_latency: float (maximum latency recorded between modules in seconds)
- min_latency: float (minimum latency recorded between modules in seconds)
- avg_latency: float (average latency between modules in seconds)
- control_frequency: float (actual control frequency achieved in Hz)
- perception_frequency: float (actual perception frequency achieved in Hz)
- task_completion_rate: float (percentage of successful task completions in repeated tests)
- resource_usage: dict (CPU, GPU, memory usage during task)
- success_per_phase: dict (success rate for each phase of the task)

**Relationships**:
- One-to-one with TaskOutcome (one performance summary per task outcome)

**Validation rules**:
- All time values must be non-negative
- Frequencies must be positive values
- Percentages must be between 0 and 100