# Python Agent to ROS Controller Bridge Contract

**Interface**: PythonAgentBridge
**Implementation**: Python with rclpy
**Version**: 1.0

## Purpose
This contract defines the interface for connecting Python-based AI/ML agents to joint trajectory controllers and differential drive controllers using the rclpy bridge with maximum 100ms latency.

## Methods

### connect_to_joint_trajectory_controller(controller_name, joint_names)
Creates a connection to a joint trajectory controller.

**Parameters**:
- controller_name (string): Name of the joint trajectory controller
- joint_names (list of string): Names of the joints controlled by this controller

**Returns**: Controller interface object

**Errors**:
- ControllerNotFoundError: If the specified controller doesn't exist
- InvalidJointNamesError: If joint_names is invalid or empty

### connect_to_differential_drive_controller(controller_name)
Creates a connection to a differential drive controller.

**Parameters**:
- controller_name (string): Name of the differential drive controller

**Returns**: Controller interface object

**Errors**:
- ControllerNotFoundError: If the specified controller doesn't exist

### send_joint_trajectory_command(controller_interface, trajectory_points)
Sends a joint trajectory command to a connected controller.

**Parameters**:
- controller_interface (object): Interface to the joint trajectory controller
- trajectory_points (list of JointTrajectoryPoint): Points defining the trajectory

**Returns**: Command result object with status and completion time

**Errors**:
- InvalidTrajectoryError: If trajectory_points are invalid
- ControllerNotReadyError: If the controller is not ready to accept commands

### send_velocity_command(controller_interface, linear_velocity, angular_velocity)
Sends a velocity command to a differential drive controller.

**Parameters**:
- controller_interface (object): Interface to the differential drive controller
- linear_velocity (float): Linear velocity in m/s
- angular_velocity (float): Angular velocity in rad/s

**Returns**: Command result object with status and completion time

**Errors**:
- InvalidVelocityError: If velocity values are out of bounds
- ControllerNotReadyError: If the controller is not ready to accept commands

### register_sensor_callback(topic_name, callback, message_type)
Registers a callback function to process sensor data from ROS topics.

**Parameters**:
- topic_name (string): Name of the sensor topic
- callback (function): Function to process sensor data
- message_type (class): ROS message type for the sensor data

**Returns**: Subscription interface object

**Errors**:
- InvalidTopicNameError: If topic_name is invalid
- InvalidCallbackError: If callback is not a callable function
- InvalidMessageTypeError: If message_type is not a valid ROS message type

## Performance Requirements
- Communication latency must be ≤ 100ms
- Control loop frequency must be ≥ 50Hz
- Sensor data processing must not introduce additional latency beyond 10ms

## Data Structures

### JointTrajectoryPoint
- positions (list of float): Joint positions
- velocities (list of float): Joint velocities
- accelerations (list of float): Joint accelerations
- time_from_start (Duration): Time from start of trajectory