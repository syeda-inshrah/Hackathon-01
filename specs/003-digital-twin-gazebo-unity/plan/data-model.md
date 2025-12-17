# Data Model: Module 2: The Digital Twin (Gazebo & Unity)

**Feature**: Module 2: The Digital Twin (Gazebo & Unity)
**Branch**: 003-digital-twin-gazebo-unity
**Created**: 2025-12-15
**Status**: Draft

## Entities

### SDFModel
Represents a Simulation Description Format model for physics simulation in Gazebo

**Fields**:
- name: string (unique identifier for the model)
- file_path: string (path to the SDF file)
- links: list of Link objects (robot links with physics properties)
- joints: list of Joint objects (robot joints with actuation properties)
- materials: list of Material objects (visual materials)
- gripper_attached: boolean (indicates if grippers are included)

**Relationships**:
- One-to-many with Link (one SDF model has multiple links)
- One-to-many with Joint (one SDF model has multiple joints)
- One-to-many with Material (one SDF model has multiple materials)

**Validation rules**:
- file_path must point to a valid SDF file
- name must be unique within the Gazebo environment
- Must have at least one link and one joint for a valid robot model
- Physics properties must be valid for Gazebo simulation

### URDFCompatibilityLayer
Provides URDF compatibility for ROS ecosystem integration

**Fields**:
- sdf_reference: SDFModel (reference to the SDF model)
- converted_urdf: string (converted URDF content or file path)
- kinematic_properties: list of KinematicProperty objects
- visual_properties: list of VisualProperty objects

**Relationships**:
- Many-to-one with SDFModel (many URDF compatibility layers map to one SDF model)

**Validation rules**:
- Must reference a valid SDFModel
- Converted URDF must be syntactically correct
- Kinematic properties must match those in the SDF model

### Link
Represents a link in an SDF model

**Fields**:
- name: string (name of the link)
- visual: Visual object (visual representation)
- collision: Collision object (collision properties)
- inertial: Inertial object (physical properties)
- parent_sdf: SDFModel (reference to the parent model)
- mass: float (mass in kg)
- friction_coefficient: float (friction coefficient for simulation)

**Relationships**:
- Many-to-one with SDFModel (multiple links belong to one SDF model)
- One-to-many with Visual (one link has one visual representation)
- One-to-many with Collision (one link has one collision property)
- One-to-many with Inertial (one link has one inertial property)

**Validation rules**:
- name must be unique within the SDF model
- Must have valid visual or collision properties
- Mass must be a positive value
- Friction coefficient must be between 0 and 1

### Joint
Represents a joint in an SDF model

**Fields**:
- name: string (name of the joint)
- type: string (joint type: revolute, continuous, prismatic, etc.)
- parent_link: string (name of the parent link)
- child_link: string (name of the child link)
- origin: Pose object (position and orientation relative to parent)
- axis: Vector object (axis of rotation or translation)
- limits: JointLimits object (joint limits if applicable)
- parent_sdf: SDFModel (reference to the parent model)
- stiffness: float (joint stiffness for simulation)
- damping: float (joint damping for simulation)

**Relationships**:
- Many-to-one with SDFModel (multiple joints belong to one SDF model)

**Validation rules**:
- name must be unique within the SDF model
- parent_link and child_link must reference valid links in the same SDF model
- type must be a valid joint type (revolute, continuous, prismatic, fixed, etc.)
- Stiffness and damping must be non-negative values

### LiDARSensor
Represents a simulated LiDAR sensor with realistic noise models

**Fields**:
- name: string (name of the sensor)
- position: Vector3 (position relative to parent link)
- orientation: Quaternion (orientation relative to parent link)
- parent_link: string (name of the link this sensor is attached to)
- sensor_type: string (type of LiDAR, e.g., "2D", "3D")
- range_min: float (minimum sensing range in meters)
- range_max: float (maximum sensing range in meters)
- resolution: float (angular resolution in degrees)
- noise_model: NoiseModel object (realistic noise characteristics)
- topic_name: string (ROS topic name for output)
- update_rate: float (sensor update rate in Hz)

**Relationships**:
- Many-to-one with SDFModel (multiple sensors belong to one SDF model)
- One-to-one with NoiseModel (one sensor has one noise model)

**Validation rules**:
- parent_link must reference a valid link in the same SDFModel
- range_min must be less than range_max
- resolution must be positive
- update_rate must be positive
- topic_name must follow ROS topic naming conventions

### DepthCameraSensor
Represents a simulated depth camera sensor with realistic noise models

**Fields**:
- name: string (name of the sensor)
- position: Vector3 (position relative to parent link)
- orientation: Quaternion (orientation relative to parent link)
- parent_link: string (name of the link this sensor is attached to)
- camera_model: string (model of camera, e.g., "RealSense-like")
- resolution_width: int (image width in pixels)
- resolution_height: int (image height in pixels)
- fov_horizontal: float (horizontal field of view in degrees)
- fov_vertical: float (vertical field of view in degrees)
- noise_model: NoiseModel object (realistic noise characteristics)
- depth_topic: string (ROS topic name for depth image output)
- rgb_topic: string (ROS topic name for RGB image output)
- camera_info_topic: string (ROS topic name for camera info output)
- update_rate: float (sensor update rate in Hz)

**Relationships**:
- Many-to-one with SDFModel (multiple sensors belong to one SDF model)
- One-to-one with NoiseModel (one sensor has one noise model)

**Validation rules**:
- parent_link must reference a valid link in the same SDFModel
- Resolution values must be positive integers
- FOV values must be between 0 and 180 degrees
- update_rate must be positive
- Topic names must follow ROS topic naming conventions

### NoiseModel
Represents realistic noise characteristics for sensor simulation

**Fields**:
- model_type: string (type of noise model, e.g., "gaussian", "uniform")
- mean: float (mean of the noise distribution)
- standard_deviation: float (standard deviation of the noise distribution)
- bias: float (systematic bias in the measurements)
- params: dict (additional parameters specific to the noise model)

**Validation rules**:
- standard_deviation must be non-negative
- model_type must be a supported noise model type

### UnityVisualizer
Represents the Unity visualizer component of the digital twin

**Fields**:
- name: string (name of the visualizer instance)
- connection_endpoint: string (IP address and port of the ROS-TCP-Endpoint)
- synchronization_latency: float (target synchronization latency in ms)
- visualization_quality: string ("low", "medium", "high", "ultra")
- robot_model_path: string (path to the Unity-compatible robot model)
- camera_settings: dict (settings for the visualization camera)
- rendering_options: dict (rendering quality settings)

**Validation rules**:
- connection_endpoint must be a valid IP:port combination
- synchronization_latency must be positive
- visualization_quality must be one of the predefined values
- robot_model_path must point to a valid Unity asset

### DigitalTwinState
Represents the synchronized state between Gazebo and Unity

**Fields**:
- timestamp: datetime (time of last synchronization)
- robot_positions: dict (mapping of joint names to position values)
- sensor_data: dict (mapping of sensor names to latest readings)
- physics_state: dict (physics properties like velocities, forces)
- visualization_sync: dict (visualization-specific synchronization data)
- last_update_duration: float (time taken for last state update in ms)
- connection_status: string ("connected", "disconnected", "degraded")

**Validation rules**:
- timestamp must be current or recent
- robot_positions values must be within joint limits
- last_update_duration must be positive
- connection_status must be one of the valid states

### GazeboPhysicsEngine
Represents the Gazebo physics simulation environment

**Fields**:
- name: string (name of the simulation instance)
- world_file: string (path to the world file)
- physics_engine: string (physics engine type, e.g., "ODE", "Bullet", "SimBody")
- gravity_x: float (gravity in X direction, m/s²)
- gravity_y: float (gravity in Y direction, m/s²)
- gravity_z: float (gravity in Z direction, m/s², typically -9.81)
- max_step_size: float (maximum simulation step size)
- real_time_factor: float (target real-time factor)
- update_rate: float (world update rate in Hz)
- contact_surface_layer: float (contact surface layer thickness)

**Validation rules**:
- world_file must point to a valid Gazebo world file
- gravity values must be reasonable for Earth-like simulation
- max_step_size must be positive
- real_time_factor must be positive
- update_rate must be positive

### ROSCommunicationInterface
Represents the ROS communication for the digital twin

**Fields**:
- node_name: string (name of the ROS node)
- publishers: list of Publisher objects (active publishers)
- subscribers: list of Subscriber objects (active subscribers)
- services: list of Service objects (active services)
- actions: list of Action objects (active actions)
- topic_prefix: string (prefix for all topics used by this interface)
- heartbeat_interval: float (interval for heartbeat messages in seconds)

**Validation rules**:
- node_name must be unique in the ROS network
- Topic names must follow ROS naming conventions
- heartbeat_interval must be positive
- Publishers and subscribers must connect to valid topic types

### ROSEndpointBridge
Represents the ROS-TCP-Endpoint bridge for Unity communication

**Fields**:
- name: string (name of the bridge instance)
- host_ip: string (IP address of the host)
- port: int (port number for communication)
- max_connections: int (maximum allowed connections)
- buffer_size: int (size of the communication buffer)
- protocol_version: string (version of the protocol being used)
- encryption_enabled: boolean (whether communication is encrypted)
- timeout: float (timeout for communications in seconds)

**Validation rules**:
- host_ip must be a valid IP address
- port must be in valid range (1-65535)
- max_connections must be positive
- buffer_size must be positive
- timeout must be positive