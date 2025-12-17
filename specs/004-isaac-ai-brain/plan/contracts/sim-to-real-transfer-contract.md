# Sim-to-Real Transfer API Contract

**Interface**: SimToRealTransferInterface
**Implementation**: Python with Isaac Sim, ROS-TCP-Endpoint
**Version**: 1.0

## Purpose

This contract defines the API for sim-to-real transfer methodologies, including domain randomization techniques and policy transfer mechanisms that bridge simulated environments to physical robots.

## Endpoints

### POST /transfer/start_domain_randomization
Initiate domain randomization for sim-to-real transfer preparation.

**Request Body**:
```json
{
  "randomization_config": {
    "randomization_scopes": [
      {
        "scope_name": "string (e.g., \"lights\", \"materials\", \"physics\", \"textures\")",
        "parameters": {
          "light_intensity_range": {"min": "float", "max": "float"},
          "material_colors": [{"min": "[float,float,float]", "max": "[float,float,float]"}],
          "friction_coefficients": {"min": "float", "max": "float"},
          "mass_variations": {"min_percent": "float", "max_percent": "float"},
          "texture_patterns": ["string (list of texture types to randomize)"]
        }
      }
    ],
    "simulation_duration": "float (duration to run with randomization in seconds)",
    "data_collection_config": {
      "record_sensor_data": "boolean",
      "record_robot_state": "boolean", 
      "record_environment_state": "boolean",
      "output_directory": "string (where to save randomized data)"
    },
    "evaluation_metrics": [
      "string (metrics to track, e.g. \"performance_stability\", \"accuracy_degradation\")"
    ]
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "session_id": "string",
  "active_randomizations": ["string (scopes being randomized)"],
  "estimated_duration": "float (seconds)",
  "data_collection_path": "string"
}
```

**Errors**:
- InvalidRandomizationScopeError: If specified randomization scopes are not supported
- InsufficientResourcesError: If system can't handle requested randomizations
- InvalidParameterRangeError: If parameter ranges are invalid

### POST /transfer/policy_transfer
Transfer a trained policy from simulation to physical robot.

**Request Body**:
```json
{
  "policy_source": {
    "type": "string (\"neural_network\", \"rule_based\", \"classical_control\")",
    "path": "string (path to the trained policy/model file)",
    "training_environment": "string (description of sim environment used)",
    "hyperparameters": "dict (hyperparameters from training)"
  },
  "target_robot": {
    "model": "string (name/model of physical robot)",
    "capabilities": ["string (list of robot capabilities)"],
    "hardware_limits": {
      "max_velocity": "float",
      "max_acceleration": "float", 
      "torque_limits": {"min": "float", "max": "float"},
      "workspace_bounds": {"min": {"x": "float", "y": "float", "z": "float"}, "max": {"x": "float", "y": "float", "z": "float"}}
    }
  },
  "transfer_method": {
    "type": "string (\"direct_weights\", \"fine_tuning\", \"meta_learning\", \"system_identification\")",
    "adaptation_required": "boolean",
    "post_transfer_finetuning_config": {
      "epochs": "int",
      "learning_rate": "float",
      "data_requirements": "string (amount/type of real data needed)"
    }
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "transfer_session_id": "string",
  "estimated_success_probability": "float",
  "adaptation_recommendation": "string",
  "required_real_world_trials": "int",
  "expected_performance_drop": "float (in percentage)"
}
```

**Errors**:
- PolicyFormatNotSupportedError: If the policy format is not supported
- RobotCapabilityMismatchError: If target robot can't execute the policy
- TransferMethodUnsupportedError: If the requested transfer method is not available

### POST /transfer/validate_transfer_performance
Validate the performance of a transferred policy on the physical robot.

**Request Body**:
```json
{
  "transfer_session_id": "string",
  "validation_tasks": [
    {
      "task_description": "string (e.g., \"navigation_to_waypoint\", \"grasp_object\")",
      "task_parameters": "dict (specific to the task)",
      "success_criteria": {
        "metric": "string (e.g., \"accuracy\", \"completion_time\", \"energy_efficiency\")",
        "threshold": "float (minimum required value)",
        "acceptable_degradation": "float (how much worse than sim is acceptable)"
      }
    }
  ],
  "validation_duration": "float (time to run validation in seconds)",
  "sim_baseline_performance": {
    "task_name": "string",
    "average_performance": "float",
    "std_deviation": "float",
    "sample_size": "int"
  }
}
```

**Response**:
```json
{
  "success": "boolean",
  "validation_results": [
    {
      "task": "string",
      "sim_performance": "float",
      "real_performance": "float",
      "performance_gap": "float",
      "success": "boolean",
      "confidence_interval": {"lower": "float", "upper": "float"}
    }
  ],
  "overall_success_rate": "float",
  "recommendations": ["string (suggested improvements)"]
}
```

**Errors**:
- TransferSessionNotFoundError: If the specified transfer session doesn't exist
- ValidationTaskInvalidError: If validation tasks are not properly defined
- PerformanceMetricsMismatchError: If sim and real metrics are not comparable

### POST /transfer/adapt_policy
Adapt a simulation-traned policy for better performance on the physical robot.

**Request Body**:
```json
{
  "transfer_session_id": "string",
  "adaptation_method": {
    "technique": "string (\"fine_tuning\", \"domain_adaptation\", \"imitation_learning\", \"online_adaptation\")",
    "training_data_source": "string (\"real_robot_data\", \"sim_with_updated_params\", \"hybrid\")",
    "optimization_target": "string (\"performance\", \"safety\", \"efficiency\")"
  },
  "adaptation_parameters": {
    "learning_rate": "float",
    "epochs": "int",
    "batch_size": "int",
    "regularization_strength": "float"
  },
  "validation_tasks": [
    {
      "task_name": "string",
      "rewards": ["string (list of reward functions to optimize)"]
    }
  ]
}
```

**Response**:
```json
{
  "success": "boolean",
  "adapted_policy_path": "string",
  "original_performance": "float",
  "adapted_performance": "float",
  "improvement_percentage": "float",
  "adaptation_session_id": "string"
}
```

**Errors**:
- InsufficientTrainingDataError: If not enough real robot data is available
- AdaptationFailedError: If the adaptation process fails
- InvalidAdaptationMethodError: If the specified adaptation method is not supported

### GET /transfer/session_status/{session_id}
Get the status of a transfer session.

**Response**:
```json
{
  "session_id": "string",
  "status": "string (\"initialized\", \"domain_randomizing\", \"policy_transferring\", \"validating\", \"adapted\", \"completed\")",
  "progress_percent": "float",
  "estimated_remaining_time": "float",
  "last_evaluation": {
    "timestamp": "string (ISO 8601)",
    "performance_metrics": "dict",
    "issues_found": ["string"]
  },
  "logs": ["string (list of recent log messages)"]
}
```

**Errors**:
- SessionNotFoundError: If the session ID doesn't exist