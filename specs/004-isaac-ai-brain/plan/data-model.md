# Data Model: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

**Feature**: Module 3: The AI-Robot Brain (NVIDIA Isaac™)
**Branch**: 004-isaac-ai-brain
**Created**: 2025-12-15
**Status**: Draft

## Entities

### IsaacSimEnvironment
Represents an Isaac Sim environment configuration for photorealistic simulation

**Fields**:
- name: string (unique identifier for the environment)
- scene_path: string (path to the Isaac Sim scene file)
- physics_properties: PhysicsProperties object (gravity, friction, etc.)
- lighting_conditions: LightingConditions object (sun position, intensity, etc.)
- rendering_quality: string ("low", "medium", "high", "ultra")
- synthetic_data_generation: SyntheticDataConfig object (SDG configuration)
- sensors: list of Sensor objects (sensors in the environment)
- humanoid_robots: list of HumanoidRobot objects (robots in the environment)

**Relationships**:
- One-to-many with PhysicsProperties (one environment has one set of physics properties)
- One-to-many with LightingConditions (one environment has one lighting setup)
- One-to-many with SyntheticDataConfig (one environment has one SDG config)
- One-to-many with Sensor (one environment can have multiple sensors)
- One-to-many with HumanoidRobot (one environment can have multiple robots)

**Validation rules**:
- name must be unique within the Isaac Sim environment
- scene_path must point to a valid Isaac Sim scene file
- rendering_quality must be one of the predefined values ("low", "medium", "high", "ultra")

### PhysicsProperties
Defines the physical properties within an Isaac Sim environment

**Fields**:
- gravity_x: float (gravity in X direction, m/s²)
- gravity_y: float (gravity in Y direction, m/s²)
- gravity_z: float (gravity in Z direction, m/s²)
- friction_coefficient: float (default friction coefficient)
- restitution_coefficient: float (default restitution coefficient)
- damping_ratio: float (default damping ratio for joints)
- simulation_step_size: float (size of simulation steps in seconds)

**Relationships**:
- Many-to-one with IsaacSimEnvironment (multiple environments share physics properties)

**Validation rules**:
- gravity values should be within reasonable Earth gravity range (-20 to 20 m/s²)
- friction_coefficient must be ≥ 0
- restitution_coefficient must be between 0 and 1
- damping_ratio must be ≥ 0
- simulation_step_size must be > 0

### Sensor
Represents a sensor in the Isaac Sim environment

**Fields**:
- name: string (unique identifier for the sensor)
- sensor_type: string ("LiDAR", "Depth Camera", "RGB Camera", "IMU", etc.)
- position: Vector3 object (position relative to parent link)
- orientation: Quaternion object (orientation relative to parent link)
- parent_link: string (name of the link the sensor is attached to)
- config_parameters: dict (sensor-specific configuration parameters)
- enabled: boolean (whether the sensor is active)
- noise_model: NoiseModel object (realistic noise characteristics for the sensor)

**Relationships**:
- Many-to-one with IsaacSimEnvironment (multiple sensors in one environment)
- Many-to-one with NoiseModel (multiple sensors can share a noise model)

**Validation rules**:
- sensor_type must be one of the supported types
- parent_link must reference a valid link in the robot model
- position and orientation must be valid 3D coordinates
- config_parameters must be valid for the sensor type

### HumanoidRobot
Represents a humanoid robot model in the Isaac Sim environment

**Fields**:
- name: string (unique identifier for the robot)
- sdf_path: string (path to the SDF model file)
- urdf_path: string (path to the URDF compatibility file)
- joint_controllers: list of JointController objects (controllers for joints)
- sensors: list of Sensor objects (attached sensors)
- current_state: RobotState object (current pose, joint positions, etc.)
- bipedal_locomotion: LocomotionPattern object (bipedal gait patterns)

**Relationships**:
- One-to-many with JointController (one robot has multiple joint controllers)
- One-to-many with Sensor (one robot has multiple sensors)
- One-to-one with RobotState (one current state per robot)
- One-to-one with LocomotionPattern (one locomotion pattern per robot)

**Validation rules**:
- sdf_path must point to a valid SDF file
- urdf_path must point to a valid URDF file if specified
- Joint controllers must reference valid joints in the robot model
- Sensors must be attached to valid links in the robot model

### IsaacROSNode
Represents an Isaac ROS node interface for hardware acceleration

**Fields**:
- node_name: string (name of the ROS node)
- gpu_device_id: int (ID of the GPU device to use)
- acceleration_enabled: boolean (whether hardware acceleration is enabled)
- supported_algorithms: list of string (algorithms supported)
- resource_usage: ResourceUsage object (GPU memory, utilization, etc.)
- interfaces: list of ROSInterface objects (published/subscribed topics)

**Relationships**:
- One-to-many with ROSInterface (one node has multiple interfaces)
- One-to-one with ResourceUsage (one resource usage report per node)

**Validation rules**:
- node_name must follow ROS naming conventions
- gpu_device_id must be available on the system
- supported_algorithms must match available Isaac ROS packages
- resource_usage values must be accurate and monitored regularly

### ROSInterface
Represents a ROS interface (topic/service/action) used in Isaac ROS nodes

**Fields**:
- name: string (name of the interface)
- type: string ("topic", "service", "action")
- topic_type: string (message type for topics, service type for services, action type for actions)
- direction: string ("publish", "subscribe", "server", "client")
- qos_profile: QoSProfile object (quality of service settings)
- associated_algorithm: string (algorithm this interface is used for)

**Relationships**:
- Many-to-one with IsaacROSNode (multiple interfaces per node)

**Validation rules**:
- name must be a valid ROS interface name
- type must be one of "topic", "service", or "action"
- topic_type must be a valid ROS message/service/action type
- direction must be appropriate for the interface type ("publish"/"subscribe" for topics, "server"/"client" for services/actions)

### LocomotionPattern
Defines bipedal gait patterns for humanoid navigation

**Fields**:
- pattern_name: string (name of the gait pattern)
- stance_leg: string ("left" or "right") 
- foot_positions: list of Vector3 objects (foot placement positions)
- center_of_mass_margins: list of Vector3 objects (stability margin boundaries)
- support_polygon_size: float (minimum polygon size for stability)
- walking_speed: float (speed of the gait in m/s)
- step_height: float (height of steps in m)

**Relationships**:
- One-to-many with HumanoidRobot (used by multiple robots if shared)

**Validation rules**:
- stance_leg must be either "left" or "right"
- foot_positions must be valid 3D coordinates
- support_polygon_size must be positive
- walking_speed must be positive
- step_height must be positive

### SyntheticDataConfig
Configuration for synthetic data generation pipeline

**Fields**:
- output_format: string (output format, e.g., "COCO", "KITTI", "custom")
- randomization_parameters: dict (parameters for domain randomization)
- annotation_types: list of string (types of annotations to generate)
- output_directory: string (directory to save generated data)
- image_resolution: Vector2 object (resolution of generated images)
- data_augmentation: boolean (whether to apply augmentation techniques)

**Relationships**:
- One-to-one with IsaacSimEnvironment (one environment has one SDG config)

**Validation rules**:
- output_format must be supported (COCO, KITTI, etc.)
- randomization_parameters must be valid for Isaac Sim
- annotation_types must be supported by the selected output format
- output_directory must be writable

### NoiseModel
Represents realistic noise characteristics for sensor simulation

**Fields**:
- model_type: string ("gaussian", "uniform", "custom")
- mean: float (mean of the noise distribution)
- standard_deviation: float (standard deviation of the noise distribution)
- bias: float (systematic bias in the measurements)
- parameters: dict (additional parameters specific to the noise model)
- sensor_type: string (type of sensor this model applies to)

**Relationships**:
- One-to-many with Sensor (one noise model can be used by multiple sensors)

**Validation rules**:
- model_type must be supported (gaussian, uniform, custom)
- standard_deviation must be non-negative
- sensor_type must correspond to a valid sensor type

### RobotState
Represents the current state of a robot including pose, velocities, and effort

**Fields**:
- timestamp: datetime (when the state was captured)
- position: Vector3 object (position in world coordinates)
- orientation: Quaternion object (orientation in world coordinates)
- linear_velocity: Vector3 object (linear velocity in world coordinates)
- angular_velocity: Vector3 object (angular velocity in world coordinates)
- joint_positions: dict (mapping of joint names to positions)
- joint_velocities: dict (mapping of joint names to velocities)
- joint_efforts: dict (mapping of joint names to efforts)
- center_of_mass: Vector3 object (center of mass position)

**Relationships**:
- One-to-one with HumanoidRobot (one current state per robot)

**Validation rules**:
- timestamp must be current or recent
- position and orientation must be valid 3D coordinates
- joint positions values must be within their joint limits
- center_of_mass must be physically plausible given the joint positions