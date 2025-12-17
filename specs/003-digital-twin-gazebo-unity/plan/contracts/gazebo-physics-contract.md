# Gazebo Physics Interface Contract

**Interface**: GazeboPhysicsInterface
**Implementation**: Python with gazebo_ros_pkgs
**Version**: 1.0

## Purpose
This contract defines the interface for interacting with Gazebo physics simulation environment, including model spawning, physics properties configuration, and simulation control.

## Methods

### spawn_sdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
Spawns an SDF model in the Gazebo environment.

**Parameters**:
- model_name (string): Name of the model to be spawned
- model_xml (string): XML string containing the SDF model description
- robot_namespace (string): Namespace for the spawned model
- initial_pose (geometry_msgs.Pose): Initial pose of the model
- reference_frame (string): Reference frame for the pose

**Returns**: SpawnModelResponse object with success status and status message

**Errors**:
- InvalidSDFError: If model_xml is not a valid SDF string
- SpawnModelError: If the model cannot be spawned in Gazebo

### set_physics_properties(time_step, max_update_rate, gravity_x, gravity_y, gravity_z, ode_config)
Sets the physics properties for the Gazebo simulation.

**Parameters**:
- time_step (float): Time step for the physics simulation
- max_update_rate (float): Maximum update rate for the physics engine
- gravity_x (float): Gravity in X direction (m/s²)
- gravity_y (float): Gravity in Y direction (m/s²)
- gravity_z (float): Gravity in Z direction (m/s²)
- ode_config (gazebo.msgs.ODEPhysics): ODE physics configuration

**Returns**: Boolean indicating success or failure

**Errors**:
- InvalidPhysicsPropertiesError: If physics properties are out of valid range

### get_link_state(link_name, reference_frame)
Retrieves the current state of a specific link.

**Parameters**:
- link_name (string): Name of the link to query
- reference_frame (string): Reference frame for the state

**Returns**: LinkState object with position, orientation, velocity, and force data

**Errors**:
- LinkNotFoundError: If the specified link doesn't exist

### apply_joint_effort(joint_name, effort, start_time, duration)
Applies an effort to a specific joint for a given duration.

**Parameters**:
- joint_name (string): Name of the joint
- effort (float): Effort to be applied
- start_time (Time): Time to start applying effort
- duration (Duration): Duration to apply the effort

**Returns**: Boolean indicating if effort was successfully applied

**Errors**:
- JointNotFoundError: If the specified joint doesn't exist
- InvalidEffortError: If the effort value is invalid

## Performance Requirements
- All operations must complete within 100ms for visualization updates
- Control loop operations must achieve minimum 50Hz frequency (20ms per update)
- Physics simulation updates should maintain 10ms latency for control operations