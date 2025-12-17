# Isaac Sim Environment API Contract

**Interface**: IsaacSimEnvironment
**Implementation**: Python with Isaac Sim Python API
**Version**: 1.0

## Purpose

This contract defines the API for interacting with Isaac Sim environments, including configuration for photorealistic rendering, synthetic data generation, and physics simulation.

## Endpoints

### POST /environments/create
Create a new Isaac Sim environment with specified configuration.

**Request Body**:
```json
{
  "name": "string (unique identifier for the environment)",
  "scene_path": "string (path to the Isaac Sim scene file)",
  "physics_properties": {
    "gravity_x": "float (gravity in X direction, m/s²)",
    "gravity_y": "float (gravity in Y direction, m/s²)", 
    "gravity_z": "float (gravity in Z direction, m/s²)",
    "friction_coefficient": "float (default friction coefficient)",
    "restitution_coefficient": "float (default restitution coefficient)",
    "damping_ratio": "float (default damping ratio for joints)",
    "simulation_step_size": "float (size of simulation steps in seconds)"
  },
  "lighting_conditions": {
    "sun_position": {"x": "float", "y": "float", "z": "float"},
    "intensity": "float (sunlight intensity)",
    "environment_lighting": "string (preset lighting configuration)"
  },
  "rendering_quality": "string (\"low\", \"medium\", \"high\", \"ultra\")",
  "synthetic_data_generation": {
    "enabled": "boolean",
    "output_format": "string (\"COCO\", \"KITTI\", \"custom\")",
    "annotation_types": ["string (types of annotations to generate)"]
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "environment_id": "string",
  "message": "string"
}
```

**Errors**:
- InvalidPhysicsPropertiesError: If physics properties are out of valid range
- InvalidScenePathError: If scene path doesn't point to valid Isaac Sim scene
- RenderingQualityNotSupportedError: If rendering quality isn't supported

### GET /environments/{environment_id}/status
Get the status of an Isaac Sim environment.

**Response**:
```json
{
  "environment_id": "string",
  "status": "string (\"running\", \"paused\", \"stopped\", \"error\")",
  "simulation_time": "float (current simulation time in seconds)",
  "real_time_factor": "float (ratio of simulation time to real time)",
  "entities_count": "int (number of entities in the environment)"
}
```

### PUT /environments/{environment_id}/configure
Update configuration of an existing Isaac Sim environment.

**Request Body**:
```json
{
  "physics_properties": {
    "gravity_x": "float (gravity in X direction, m/s²)",
    "gravity_y": "float (gravity in Y direction, m/s²)",
    "gravity_z": "float (gravity in Z direction, m/s²)",
    "friction_coefficient": "float (default friction coefficient)",
    "restitution_coefficient": "float (default restitution coefficient)",
    "damping_ratio": "float (default damping ratio for joints)",
    "simulation_step_size": "float (size of simulation steps in seconds)"
  },
  "lighting_conditions": {
    "sun_position": {"x": "float", "y": "float", "z": "float"},
    "intensity": "float (sunlight intensity)",
    "environment_lighting": "string (preset lighting configuration)"
  },
  "rendering_quality": "string (\"low\", \"medium\", \"high\", \"ultra\")"
}
```

**Response**:
```json
{
  "success": "boolean",
  "message": "string"
}
```

**Errors**:
- EnvironmentNotFoundError: If the environment doesn't exist
- InvalidConfigurationError: If the new configuration is invalid

### POST /environments/{environment_id}/generate_synthetic_data
Trigger synthetic data generation in an environment.

**Request Body**:
```json
{
  "duration": "float (how long to generate data, in seconds)",
  "output_directory": "string (where to save the generated data)",
  "cameras_to_use": ["string (names of cameras to use)"],
  "sensors_to_use": ["string (names of sensors to use)"],
  "annotation_types": ["string (types of annotations to include)"]
}
```

**Response**:
```json
{
  "success": "boolean",
  "generation_job_id": "string",
  "estimated_completion_time": "float (in seconds)",
  "output_path": "string"
}
```

**Errors**:
- EnvironmentNotRenderingError: If the environment isn't set up for rendering
- InvalidOutputDirectoryError: If the output directory isn't writable
- InvalidAnnotationTypeError: If requested annotation types aren't supported