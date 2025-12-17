# Data Model: Module 1: The Robotic Nervous System (ROS 2)

**Feature**: Module 1: The Robotic Nervous System (ROS 2)
**Branch**: 002-ros2-nervous-system
**Created**: 2025-12-15
**Status**: Draft

## Entities

### ROS2Node
Represents a ROS 2 node implementation in Python

**Fields**:
- name: string (unique identifier for the node)
- namespace: string (optional namespace for the node)
- node_handle: rclpy.node.Node (the actual node instance)
- publishers: list of Publisher objects
- subscribers: list of Subscriber objects
- services: list of Service objects
- actions: list of Action objects

**Relationships**:
- One-to-many with Publisher (a node can have multiple publishers)
- One-to-many with Subscriber (a node can have multiple subscribers)
- One-to-many with Service (a node can have multiple services)
- One-to-many with Action (a node can have multiple action servers/clients)

**Validation rules**:
- name must be unique within the ROS graph
- node_handle must be a valid rclpy node instance
- namespace must follow ROS naming conventions if specified

### Publisher
Represents a ROS 2 publisher for topic-based communication

**Fields**:
- topic_name: string (name of the topic to publish to)
- message_type: string (ROS message type)
- qos_profile: QoSProfile object (quality of service settings)
- node_ref: ROS2Node (reference to the parent node)

**Relationships**:
- Many-to-one with ROS2Node (multiple publishers belong to one node)

**Validation rules**:
- topic_name must follow ROS naming conventions
- message_type must be a valid ROS 2 message type

### Subscriber
Represents a ROS 2 subscriber for topic-based communication

**Fields**:
- topic_name: string (name of the topic to subscribe to)
- message_type: string (ROS message type)
- callback_function: function (function to call when message received)
- qos_profile: QoSProfile object (quality of service settings)
- node_ref: ROS2Node (reference to the parent node)

**Relationships**:
- Many-to-one with ROS2Node (multiple subscribers belong to one node)

**Validation rules**:
- topic_name must follow ROS naming conventions
- message_type must be a valid ROS 2 message type
- callback_function must be a callable function

### ServiceServer
Represents a ROS 2 service server for request-response communication

**Fields**:
- service_name: string (name of the service)
- service_type: string (ROS service type)
- callback_function: function (function to handle service requests)
- node_ref: ROS2Node (reference to the parent node)

**Relationships**:
- Many-to-one with ROS2Node (multiple service servers belong to one node)

**Validation rules**:
- service_name must follow ROS naming conventions
- service_type must be a valid ROS 2 service type

### ServiceClient
Represents a ROS 2 service client for requesting services

**Fields**:
- service_name: string (name of the service to call)
- service_type: string (ROS service type)
- node_ref: ROS2Node (reference to the parent node)

**Relationships**:
- Many-to-one with ROS2Node (multiple service clients belong to one node)

**Validation rules**:
- service_name must follow ROS naming conventions
- service_type must be a valid ROS 2 service type

### ActionServer
Represents a ROS 2 action server for long-running tasks

**Fields**:
- action_name: string (name of the action)
- action_type: string (ROS action type)
- callback_functions: dict (functions for different action callbacks)
- node_ref: ROS2Node (reference to the parent node)

**Relationships**:
- Many-to-one with ROS2Node (multiple action servers belong to one node)

**Validation rules**:
- action_name must follow ROS naming conventions
- action_type must be a valid ROS 2 action type

### ActionClient
Represents a ROS 2 action client for requesting long-running tasks

**Fields**:
- action_name: string (name of the action to call)
- action_type: string (ROS action type)
- node_ref: ROS2Node (reference to the parent node)

**Relationships**:
- Many-to-one with ROS2Node (multiple action clients belong to one node)

**Validation rules**:
- action_name must follow ROS naming conventions
- action_type must be a valid ROS 2 action type

### URDFModel
Represents a URDF file for humanoid robot description

**Fields**:
- name: string (name of the robot)
- file_path: string (path to the URDF file)
- links: list of Link objects (robot links)
- joints: list of Joint objects (robot joints)
- materials: list of Material objects (visual materials)
- gazebo_extensions: list of GazeboExtension objects (simulation extensions)

**Relationships**:
- One-to-many with Link (one URDF model has multiple links)
- One-to-many with Joint (one URDF model has multiple joints)
- One-to-many with Material (one URDF model has multiple materials)
- One-to-many with GazeboExtension (one URDF model has multiple simulation extensions)

**Validation rules**:
- file_path must point to a valid URDF file
- name must be unique
- Must have at least one link and one joint for a valid robot model

### Link
Represents a link in a URDF robot model

**Fields**:
- name: string (name of the link)
- visual: Visual object (visual representation)
- collision: Collision object (collision properties)
- inertial: Inertial object (physical properties)
- parent_urdf: URDFModel (reference to the parent model)

**Relationships**:
- Many-to-one with URDFModel (multiple links belong to one URDF model)

**Validation rules**:
- name must be unique within the URDF model
- Must have valid visual or collision properties

### Joint
Represents a joint in a URDF robot model

**Fields**:
- name: string (name of the joint)
- type: string (joint type: revolute, continuous, prismatic, etc.)
- parent_link: string (name of the parent link)
- child_link: string (name of the child link)
- origin: Pose object (position and orientation relative to parent)
- axis: Vector object (axis of rotation or translation)
- limits: JointLimits object (joint limits if applicable)
- parent_urdf: URDFModel (reference to the parent model)

**Relationships**:
- Many-to-one with URDFModel (multiple joints belong to one URDF model)

**Validation rules**:
- name must be unique within the URDF model
- parent_link and child_link must reference valid links in the same URDF model
- type must be a valid joint type (revolute, continuous, prismatic, fixed, etc.)

### PythonAgent
Represents an AI/ML control algorithm implemented in Python

**Fields**:
- name: string (name of the agent)
- algorithm_type: string (type of algorithm: RL, NN, path-planning, etc.)
- framework: string (ML framework: TensorFlow, PyTorch, etc.)
- input_topics: list of string (topics the agent subscribes to)
- output_topics: list of string (topics the agent publishes to)
- control_frequency: float (frequency of control loop in Hz)
- latency_requirement: float (maximum allowed latency in ms)

**Relationships**:
- One-to-many with ROS2Node (an agent may control multiple nodes)

**Validation rules**:
- control_frequency must be ≥ 50Hz
- latency_requirement must be ≤ 100ms
- algorithm_type must be a valid AI/ML algorithm type