# API Contract: Perception Pipeline Interface

**Interface**: PerceptionPipeline
**Service**: Module 4 - Integrated Humanoid Task Execution
**Version**: 1.0

## Purpose

This contract defines the API interface for the perception pipeline that handles object detection, 3D pose estimation, and sensory data processing for the fetch-and-place task. The pipeline uses Isaac ROS packages for hardware-accelerated processing.

## Endpoints

### POST /perception/localize_object
Detects and localizes objects in the environment using Isaac ROS VSLAM and sensor data.

**Request Body**:
```json
{
  "request_id": "string (unique identifier for this request)",
  "object_type": "string (type of object to detect: 'cube', 'cylinder', 'sphere', etc.)",
  "region_of_interest": {
    "center": {
      "x": "float (x coordinate of region center)",
      "y": "float (y coordinate of region center)",
      "z": "float (z coordinate of region center)"
    },
    "dimensions": {
      "x": "float (width of region in meters)",
      "y": "float (depth of region in meters)",
      "z": "float (height of region in meters)"
    }
  },
  "sensor_config": {
    "depth_camera_enabled": "boolean (whether to use depth camera data)",
    "lidar_enabled": "boolean (whether to use LiDAR data)",
    "confidence_threshold": "float (minimum confidence for detections, 0.0-1.0)",
    "max_detections": "int (maximum number of objects to return)"
  },
  "coordinate_frame": "string (reference frame for returned poses, default 'base_link')"
}
```

**Response**:
```json
{
  "success": "boolean",
  "request_id": "string",
  "timestamp": "string (ISO 8601 timestamp when detection was completed)",
  "detections": [
    {
      "object_id": "string (unique identifier of detected object)",
      "object_type": "string (type of object detected)",
      "pose": {
        "position": {"x": "float", "y": "float", "z": "float"},
        "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
      },
      "confidence": "float (confidence score between 0.0 and 1.0)",
      "bbox_2d": {
        "x_min": "int (2D bounding box coordinates)",
        "y_min": "int",
        "x_max": "int",
        "y_max": "int"
      },
      "bbox_3d": {
        "center": {"x": "float", "y": "float", "z": "float"},
        "extent": {"x": "float", "y": "float", "z": "float"}
      }
    }
  ],
  "processing_time_ms": "float (time taken for processing in milliseconds)",
  "error": "string (if success is false, description of the error)"
}
```

**Errors**:
- InvalidObjectTypeError: If the requested object type is not supported
- SensorNotAvailableError: If requested sensors are not available
- ProcessingTimeoutError: If the processing takes longer than allowed
- CalibrationError: If sensor calibration is invalid for accurate pose estimation

---

### POST /perception/estimate_pose
Estimates the 3D pose of a specific object using fused sensor data.

**Request Body**:
```json
{
  "request_id": "string (unique identifier for this request)",
  "object_image_coords": {
    "u": "int (2D pixel column coordinate)",
    "v": "int (2D pixel row coordinate)"
  },
  "depth_value": "float (depth value in meters at image coordinates)",
  "sensor_frame": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "coordinate_frame": "string (output frame for pose, default 'map')"
}
```

**Response**:
```json
{
  "success": "boolean",
  "request_id": "string",
  "timestamp": "string (ISO 8601 timestamp)",
  "object_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
    "position_covariance": ["float (3x3 covariance matrix as 9-element array)"]
  },
  "confidence": "float (confidence in the pose estimate)",
  "processing_time_ms": "float (time taken for processing)",
  "error": "string (if success is false, error description)"
}
```

**Errors**:
- InvalidCoordinatesError: If image coordinates are outside sensor bounds
- InvalidDepthError: If depth value is outside sensor range
- CoordinateFrameError: If specified frame doesn't exist in the transform tree
- ProcessingFailureError: If pose estimation fails due to invalid inputs

---

### GET /perception/capabilities
Gets the current capabilities of the perception system.

**Response**:
```json
{
  "available_object_types": ["string (list of detectable object types)"],
  "sensor_availability": {
    "depth_camera": "boolean",
    "lidar": "boolean",
    "rgb_camera": "boolean",
    "imu": "boolean",
    "joint_encoders": "boolean"
  },
  "performance_metrics": {
    "object_detection_rate": "float (max detections per second)",
    "pose_estimation_rate": "float (max pose estimations per second)",
    "average_processing_time": "float (average time for processing in ms)",
    "gpu_compute_load": "float (current GPU utilization for perception)"
  },
  "calibration_status": {
    "depth_camera": "string ('calibrated', 'uncalibrated', 'needs_calibration')",
    "lidar": "string ('calibrated', 'uncalibrated', 'needs_calibration')",
    "extrinsic_calibration": "string ('valid', 'invalid', 'missing')"
  }
}
```

**Errors**:
- SystemNotReadyError: If perception system is not initialized

---

### POST /perception/register_object
Registers a new object type for detection if it's not already available.

**Request Body**:
```json
{
  "object_class": "string (class name for the new object)",
  "training_data_path": "string (path to training data for recognition)",
  "template_model": {
    "visual_features": ["string (list of visual features for recognition)"],
    "geometric_constraints": {
      "min_size": {"x": "float", "y": "float", "z": "float"},
      "max_size": {"x": "float", "y": "float", "z": "float"},
      "shape_model": "string (geometric shape like 'box', 'cylinder', 'sphere')"
    }
  },
  "sensor_config": {
    "required_sensors": ["string (list of required sensors)"],
    "minimum_requirements": {
      "depth_accuracy": "float (minimum required depth accuracy in meters)",
      "viewing_angle": "float (minimum required viewing angle in radians)"
    }
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "object_class": "string",
  "registration_id": "string (ID for tracking this registration)",
  "estimated_training_time": "float (estimated time in seconds for training)",
  "status_url": "string (URL to check registration status)",
  "message": "string (confirmation or status message)"
}
```

**Errors**:
- InsufficientTrainingDataError: If provided training data is inadequate
- UnsupportedObjectClassError: If the object class is fundamentally unsupported
- SensorRequirementsNotMetError: If required sensors are not available
- TrainingModelError: If the training process fails