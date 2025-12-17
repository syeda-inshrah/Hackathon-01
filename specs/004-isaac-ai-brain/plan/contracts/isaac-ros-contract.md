# Isaac ROS API Contract

**Interface**: IsaacROSNode
**Implementation**: Python with ISAAC_ROS packages
**Version**: 1.0

## Purpose

This contract defines the API for Isaac ROS nodes, which provide hardware-accelerated perception and processing using NVIDIA GPUs for tasks like VSLAM and sensor processing.

## Endpoints

### POST /nodes/create
Initialize a new Isaac ROS node for hardware-accelerated processing.

**Request Body**:
```json
{
  "node_name": "string (name of the ROS node)",
  "gpu_device_id": "int (ID of the GPU device to use, default 0)",
  "supported_algorithms": ["string (algorithms the node will support)"],
  "required_packages": ["string (required Isaac ROS packages)"],
  "enabled": "boolean (whether the node starts active)"
}
```

**Response**:
```json
{
  "success": "boolean",
  "node_id": "string (unique identifier for the node)",
  "gpu_utilization": "float (initial GPU utilization percentage)",
  "supported_algorithms": ["string"],
  "message": "string"
}
```

**Errors**:
- GPUUnavailableError: If the specified GPU device is not available
- PackageNotInstalledError: If required Isaac ROS packages are missing
- InvalidAlgorithmError: If requested algorithms are not supported

### POST /nodes/{node_id}/start_algorithm
Start a specific hardware-accelerated algorithm on the Isaac ROS node.

**Request Body**:
```json
{
  "algorithm_name": "string (name of the algorithm, e.g., \"visual_slam\", \"image_pipeline\", \"apriltag_detection\")",
  "configuration": {
    "input_topics": ["string (list of input topic names)"],
    "output_topics": ["string (list of output topic names)"],
    "processing_parameters": "dict (algorithm-specific parameters)",
    "acceleration_parameters": {
      "tensor_cores_enabled": "boolean",
      "mixed_precision": "boolean",
      "memory_reservation_mb": "int"
    }
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "algorithm_instance_id": "string",
  "gpu_memory_used_mb": "float",
  "processing_rate_hz": "float",
  "status": "string (\"running\", \"failed\", \"initializing\")"
}
```

**Errors**:
- AlgorithmNotSupportedError: If the requested algorithm isn't supported
- InsufficientGPUMemoryError: If GPU doesn't have enough memory for processing
- InvalidConfigurationError: If configuration parameters are invalid

### GET /nodes/{node_id}/resource_usage
Get resource usage information for the Isaac ROS node.

**Response**:
```json
{
  "node_id": "string",
  "gpu_utilization_percent": "float",
  "gpu_memory_used_mb": "float",
  "gpu_memory_total_mb": "float",
  "processing_rates": {
    "algorithm_name": "float (actual processing rate in Hz)"
  },
  "temperature_celsius": "float",
  "timestamp": "string (ISO 8601 formatted timestamp)"
}
```

**Errors**:
- NodeNotActiveError: If the node is not currently active

### PUT /nodes/{node_id}/update_configuration
Update the configuration of an Isaac ROS node.

**Request Body**:
```json
{
  "new_gpu_device_id": "int (optional, new GPU device ID)",
  "new_acceleration_settings": {
    "tensor_cores_enabled": "boolean",
    "mixed_precision": "boolean",
    "memory_reservation_mb": "int"
  },
  "new_processing_parameters": "dict (algorithm-specific parameters)"
}
```

**Response**:
```json
{
  "success": "boolean",
  "updated_node_id": "string",
  "new_gpu_utilization_percent": "float",
  "message": "string"
}
```

### GET /nodes/{node_id}/supported_algorithms
Get a list of supported algorithms for the Isaac ROS node.

**Response**:
```json
{
  "node_id": "string",
  "supported_algorithms": [
    {
      "name": "string",
      "description": "string",
      "min_gpu_memory_mb": "int",
      "typical_processing_rate_hz": "float",
      "input_formats": ["string"],
      "output_formats": ["string"]
    }
  ]
}
```