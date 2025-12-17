---
title: "Week 3 - LLM Task Planning and Action Execution"
sidebar_label: "Week 3: LLM Task Planning"
description: "Understanding LLM integration for cognitive planning and task execution in robotics"
keywords: [llm, large language model, task planning, cognitive robotics, gpt, robotics ai]
---

# Week 3: LLM Task Planning and Action Execution

## Learning Outcomes

By the end of this week, you will be able to:
- Integrate Large Language Models (LLMs) for task planning in robotics
- Translate natural language commands into executable robot actions
- Implement cognitive planning systems using LLMs
- Design action execution frameworks that work with LLM outputs
- Create validation systems for LLM-generated plans
- Handle errors and ambiguity in LLM-based planning

## Introduction to LLM-Based Cognitive Planning

Large Language Models (LLMs) like GPT-4, Claude, and others have demonstrated remarkable capabilities for understanding, reasoning, and generating natural language. In robotics, these models can serve as cognitive planners that interpret high-level user commands and decompose them into sequences of executable robot actions.

### Benefits of LLMs in Robotics

- **Natural Language Interface**: Users can interact using everyday language
- **Common-Sense Reasoning**: LLMs can infer common-sense knowledge
- **Flexible Planning**: Can adapt plans to new situations without reprogramming
- **Learning from Instruction**: Can follow new procedures described in text

### Challenges with LLMs in Robotics

- **Hallucination**: LLMs may generate incorrect or unsafe actions
- **Lack of Real-World Grounding**: Models don't inherently understand physical constraints
- **Stochasticity**: The same prompt may generate different responses
- **Latency**: API calls can introduce delays in real-time systems

## LLM Integration Architecture

### Cognitive Planning Framework

The integration of LLMs into robotic systems typically follows this architecture:

```
Natural Language Command
         ↓
    LLM Task Planner
         ↓
  Robot Action Sequence
         ↓
   Action Execution
         ↓
    Perception Feedback
```

### Planning with Language Models

LLMs can be used for several types of planning:

1. **Long-term Planning**: Multi-step task decomposition
2. **Short-term Planning**: Immediate action selection
3. **Contingency Planning**: Planning for potential failure scenarios

```python title="LLM-Based Task Planning"
import openai
import json
import time
from typing import List, Dict, Any

class LLMBot:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model
    
    def generate_plan(self, command: str, robot_capabilities: List[str], environment_info: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate a task plan based on command and robot capabilities
        """
        prompt = f"""
        You are a robotic task planner. Based on the user command, the robot's capabilities, 
        and the environment information, generate a step-by-step plan for the robot to execute.
        
        User Command: "{command}"
        
        Robot Capabilities: {', '.join(robot_capabilities)}
        
        Environment Information: {json.dumps(environment_info)}
        
        Generate a plan in the following JSON format:
        {{
            "thoughts": "Your reasoning about the task",
            "plan": [
                {{
                    "step": 1,
                    "action": "action_name",
                    "description": "What the robot should do",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "expected_outcome": "What should happen"
                }}
            ]
        }}
        
        Actions should be from the robot's capabilities. Be specific and practical.
        For navigation, include coordinates or object names.
        For manipulation, include object names and poses.
        """
        
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,  # Lower temperature for more consistent output
                max_tokens=1000
            )
            
            content = response.choices[0].message['content']
            plan = json.loads(content)
            return plan
        except Exception as e:
            print(f"Error generating plan: {e}")
            return {"thoughts": "Error generating plan", "plan": []}

class RobotActionExecutor:
    def __init__(self):
        # Simulate robot capabilities
        self.capabilities = [
            "navigate_to_object",
            "navigate_to_location",
            "grasp_object",
            "release_object",
            "detect_object",
            "orient_to_object",
            "wait",
            "find_placement_surface"
        ]
        
        # Simulate environment knowledge
        self.environment_objects = {
            "cup": {"x": 1.2, "y": 0.5, "z": 0.8, "color": "blue"},
            "book": {"x": 0.8, "y": -0.3, "z": 0.8, "color": "red"},
            "table": {"x": 0.0, "y": 0.0, "z": 0.0, "type": "furniture"},
            "chair": {"x": -0.5, "y": 0.7, "z": 0.0, "type": "furniture"}
        }
    
    def execute_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute the specified action (simulated)
        """
        print(f"Executing action: {action['action']}")
        print(f"Parameters: {action.get('parameters', {})}")
        
        # Simulate action execution
        time.sleep(0.5)  # Simulate time for action
        
        # In a real system, this would interface with actual robot controllers
        return True

class LLMBasedRobotPlanner:
    def __init__(self, llm_api_key: str):
        self.llm_bot = LLMBot(llm_api_key)
        self.action_executor = RobotActionExecutor()
    
    def execute_command(self, command: str) -> bool:
        """
        Execute a command from start to finish
        """
        print(f"Processing command: {command}")
        
        # Generate plan using LLM
        plan = self.llm_bot.generate_plan(
            command=command,
            robot_capabilities=self.action_executor.capabilities,
            environment_info=self.action_executor.environment_objects
        )
        
        print(f"Generated plan: {plan['thoughts']}")
        
        if not plan.get('plan'):
            print("No plan generated, stopping execution")
            return False
        
        # Execute plan step by step
        for step_info in plan['plan']:
            print(f"Step {step_info['step']}: {step_info['description']}")
            
            success = self.action_executor.execute_action(step_info)
            if not success:
                print(f"Failed to execute step {step_info['step']}")
                return False
        
        print("Command execution completed successfully")
        return True

# Example usage (API key required for actual execution)
# planner = LLMBasedRobotPlanner("your-openai-api-key")
# success = planner.execute_command("Pick up the blue cup and place it on the table")
```

## Context Awareness and World Modeling

### Maintaining World State

LLMs need to understand the current state of the world to reason about tasks effectively. This requires maintaining a consistent world model:

```python title="World State Management"
class WorldState:
    def __init__(self):
        self.objects = {}  # {object_id: {properties}}
        self.robot_state = {
            'position': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0},
            'holding': None,  # Object the robot is currently holding
            'battery_level': 100
        }
        self.rooms = {}  # {room_id: {properties}}
        self.update_history = []  # Track changes for debugging
    
    def update_object(self, obj_id: str, properties: Dict[str, Any]):
        """Update an object's properties in the world model"""
        if obj_id not in self.objects:
            self.objects[obj_id] = {}
        self.objects[obj_id].update(properties)
        self._log_update(f"Updated object {obj_id}: {properties}")
    
    def remove_object(self, obj_id: str):
        """Remove an object from the world model"""
        if obj_id in self.objects:
            del self.objects[obj_id]
            self._log_update(f"Removed object {obj_id}")
    
    def update_robot_state(self, new_state: Dict[str, Any]):
        """Update robot's state"""
        self.robot_state.update(new_state)
        self._log_update(f"Updated robot state: {new_state}")
    
    def get_context_for_llm(self) -> str:
        """Get world state formatted for LLM consumption"""
        context = {
            "robot": self.robot_state,
            "objects": self.objects,
            "rooms": self.rooms,
            "current_time": time.time()
        }
        return json.dumps(context, indent=2)
    
    def _log_update(self, description: str):
        """Log updates for debugging"""
        self.update_history.append({
            'timestamp': time.time(),
            'description': description
        })

# Example usage
world_state = WorldState()

# Update world state based on perception
world_state.update_object("blue_cup", {
    "x": 1.2, "y": 0.5, "z": 0.8, 
    "color": "blue", 
    "status": "available"
})

world_state.update_robot_state({
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "holding": None
})

context_for_llm = world_state.get_context_for_llm()
print("World state for LLM:")
print(context_for_llm)
```

## Validation and Safety Mechanisms

### Plan Validation Pipeline

LLM-generated plans need to be validated before execution to ensure safety:

```python title="Plan Validation System"
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

class ValidationResult(Enum):
    VALID = "valid"
    INVALID = "invalid"
    NEEDS_REVISION = "needs_revision"

@dataclass
class ValidationIssue:
    severity: str  # "error", "warning", "info"
    message: str
    component: str  # "action", "object", "navigation", etc.

class PlanValidator:
    def __init__(self, world_state: WorldState):
        self.world_state = world_state
        self.known_dangerous_actions = [
            "drop_object", "move_fast", "apply_high_force"
        ]
        self.physical_constraints = {
            "max_payload": 5.0,  # kg
            "max_navigation_distance": 20.0,  # meters
            "reachable_distance": 1.0  # meters
        }
    
    def validate_plan(self, plan: List[Dict]) -> (ValidationResult, List[ValidationIssue]):
        """
        Validate a plan for safety and feasibility
        """
        issues = []
        
        for step in plan:
            action = step.get('action', '')
            params = step.get('parameters', {})
            
            # Check if action is in robot's capabilities
            if action not in self.world_state.robot_capabilities:
                issues.append(ValidationIssue(
                    severity="error",
                    message=f"Action '{action}' not supported by robot",
                    component="action"
                ))
            
            # Validate object references
            if 'object' in params or action in ['grasp_object', 'release_object']:
                obj_name = params.get('object', '')
                if obj_name and obj_name not in self.world_state.objects:
                    issues.append(ValidationIssue(
                        severity="error",
                        message=f"Object '{obj_name}' not found in environment",
                        component="object"
                    ))
            
            # Check for dangerous actions
            if action in self.known_dangerous_actions:
                issues.append(ValidationIssue(
                    severity="warning",
                    message=f"Potentially dangerous action: {action}",
                    component="action"
                ))
            
            # Validate navigation constraints
            if action == 'navigate_to_location':
                dest_x = params.get('x', 0)
                dest_y = params.get('y', 0)
                robot_pos = self.world_state.robot_state['position']
                
                distance = ((dest_x - robot_pos['x'])**2 + (dest_y - robot_pos['y'])**2)**0.5
                if distance > self.physical_constraints['max_navigation_distance']:
                    issues.append(ValidationIssue(
                        severity="error",
                        message=f"Navigation destination too far: {distance:.2f}m > {self.physical_constraints['max_navigation_distance']}m",
                        component="navigation"
                    ))
            
            # Validate payload constraints
            if action == 'grasp_object':
                obj_name = params.get('object', '')
                if obj_name in self.world_state.objects:
                    obj_weight = self.world_state.objects[obj_name].get('weight', 1.0)
                    if obj_weight > self.physical_constraints['max_payload']:
                        issues.append(ValidationIssue(
                            severity="error",
                            message=f"Object too heavy: {obj_weight}kg > {self.physical_constraints['max_payload']}kg",
                            component="manipulation"
                        ))
        
        # Determine overall validation result
        errors = [issue for issue in issues if issue.severity == "error"]
        warnings = [issue for issue in issues if issue.severity == "warning"]
        
        if errors:
            return ValidationResult.INVALID, issues
        elif warnings:
            return ValidationResult.NEEDS_REVISION, issues
        else:
            return ValidationResult.VALID, issues

class SafeLLMPlanner:
    def __init__(self, llm_api_key: str):
        self.llm_planner = LLMBasedRobotPlanner(llm_api_key)
        self.world_state = WorldState()
        self.validator = PlanValidator(self.world_state)
    
    def execute_command_with_validation(self, command: str) -> bool:
        """
        Execute command with validation
        """
        # Generate plan
        plan = self.llm_planner.llm_bot.generate_plan(
            command=command,
            robot_capabilities=self.llm_planner.action_executor.capabilities,
            environment_info=self.world_state.get_context_for_llm()
        )
        
        # Validate plan
        validation_result, issues = self.validator.validate_plan(plan.get('plan', []))
        
        # Report validation results
        if validation_result == ValidationResult.INVALID:
            print(f"Plan validation failed with {len([i for i in issues if i.severity == 'error'])} errors:")
            for issue in issues:
                print(f"  [{issue.severity.upper()}] {issue.message}")
            return False
        
        if validation_result == ValidationResult.NEEDS_REVISION:
            print(f"Plan has {len([i for i in issues if i.severity == 'warning'])} warnings:")
            for issue in issues:
                if issue.severity == "warning":
                    print(f"  [WARNING] {issue.message}")
        
        # Execute validated plan
        print("Plan validation passed. Executing...")
        return self.llm_planner.execute_command(command)
```

## Handling Ambiguity and Requests for Clarification

### Clarification System

LLMs often need to ask for clarification when commands are ambiguous:

```python title="Clarification System"
class ClarificationSystem:
    def __init__(self):
        self.ambiguity_patterns = [
            # Patterns that indicate need for clarification
            r"the.*" ,  # Refers to "the object" without specifying which
            r"it",      # Vague reference
            r"there",   # Unclear location reference
            r"over there", # Unclear location reference
        ]
        
        self.implicit_assumptions = [
            # Things the LLM might assume incorrectly
            "robot can reach any location",
            "all objects are manipulable",
            "navigation is always possible"
        ]
    
    def detect_ambiguity(self, command: str) -> List[str]:
        """
        Detect potential ambiguities in a command
        """
        import re
        ambiguities = []
        
        for pattern in self.ambiguity_patterns:
            if re.search(pattern, command, re.IGNORECASE):
                ambiguities.append(f"Ambiguous reference matching pattern: {pattern}")
        
        # Check for vague spatial references
        vague_refs = ["there", "over there", "that", "this"]
        for ref in vague_refs:
            if ref.lower() in command.lower():
                ambiguities.append(f"Vague spatial reference: {ref}")
        
        # Check for ambiguous object references
        common_objects = ["cup", "book", "bottle", "box"]
        for obj in common_objects:
            # Check if object is mentioned without specification
            if obj in command.lower():
                # Check if it's qualified (e.g., "the red cup" vs "the cup")
                # This is a simplified check
                import re
                matches = re.findall(rf"the\s+(\w+)\s+{obj}", command.lower())
                if not matches:
                    matches = re.findall(rf"a\s+(\w+)\s+{obj}", command.lower())
                if not matches:
                    # Could be ambiguous if there are multiple objects of the same type
                    ambiguities.append(f"Ambiguous object reference: '{obj}' without clear specification")
        
        return ambiguities
    
    def generate_clarification_request(self, command: str, ambiguities: List[str]) -> str:
        """
        Generate a request for clarification
        """
        if not ambiguities:
            return ""
        
        clarification_request = "I need clarification for your request: "
        clarification_request += " ".join(ambiguities)
        clarification_request += f"\n\nCould you please specify: "
        
        # Based on detected ambiguities, ask specific questions
        if any("ambiguous reference" in amb for amb in ambiguities):
            clarification_request += "Which specific object you mean. "
        
        if any("spatial reference" in amb for amb in ambiguities):
            clarification_request += "The exact location you're referring to. "
        
        return clarification_request

# Example usage
clarifier = ClarificationSystem()

test_commands = [
    "Pick up the cup",
    "Go to the table over there",
    "Move the blue cup to the left of the book",
    "Put it there"
]

for cmd in test_commands:
    ambiguities = clarifier.detect_ambiguity(cmd)
    if ambiguities:
        clarification = clarifier.generate_clarification_request(cmd, ambiguities)
        print(f"Command: '{cmd}'")
        print(f"Ambiguities: {ambiguities}")
        print(f"Clarification Request: {clarification}")
        print("-" * 50)
    else:
        print(f"Command: '{cmd}' - No detected ambiguities")
        print("-" * 30)
```

## Integration with ROS 2

### LLM Planning Node

```python title="ROS 2 LLM Planning Node"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from robotics_application_interfaces.srv import PlanExecution, TaskSpecification

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')
        
        # Initialize LLM planner
        self.llm_planner = self.initialize_llm_planner()
        self.world_state = WorldState()
        
        # Create service for planning and execution
        self.plan_service = self.create_service(
            TaskSpecification, 
            'plan_task', 
            self.plan_task_callback
        )
        
        # Create service for execution only
        self.execute_service = self.create_service(
            PlanExecution,
            'execute_plan',
            self.execute_plan_callback
        )
        
        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'llm_planning_status', 10)
        
        # Publisher for plan visualization (optional)
        self.plan_vis_pub = self.create_publisher(Pose, 'plan_visualization', 10)
        
        self.get_logger().info('LLM Planning Node initialized')
    
    def initialize_llm_planner(self):
        """Initialize the LLM-based planner (simplified)"""
        # In practice, you would initialize with API key from parameters
        import os
        api_key = os.getenv('OPENAI_API_KEY', 'dummy_key')
        return LLMBasedRobotPlanner(api_key)
    
    def plan_task_callback(self, request, response):
        """Plan a task based on natural language command"""
        command = request.command
        self.get_logger().info(f'Received task planning request: {command}')
        
        try:
            # Generate plan using LLM
            plan = self.llm_planner.llm_bot.generate_plan(
                command=command,
                robot_capabilities=self.get_robot_capabilities(),
                environment_info=self.world_state.get_context_for_llm()
            )
            
            # Validate plan
            validator = PlanValidator(self.world_state)
            validation_result, issues = validator.validate_plan(plan.get('plan', []))
            
            if validation_result == ValidationResult.INVALID:
                response.success = False
                response.message = f"Plan validation failed: {'; '.join(i.message for i in issues)}"
                return response
            
            # Convert plan to response format (simplified)
            # In practice, you'd convert the LLM output to a ROS-compatible format
            response.plan = str(plan)
            response.success = True
            response.message = "Plan generated successfully"
            
            # Publish status
            status_msg = String()
            status_msg.data = f"Generated plan for: {command}"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            response.success = False
            response.message = f"Error generating plan: {str(e)}"
            self.get_logger().error(f'Error in plan_task_callback: {str(e)}')
        
        return response
    
    def execute_plan_callback(self, request, response):
        """Execute a pre-generated plan"""
        plan_str = request.plan
        self.get_logger().info('Received plan execution request')
        
        try:
            # In practice, execute the plan using robot controllers
            # For this example, we'll simulate execution
            plan_success = True  # Simulated execution
            
            response.success = plan_success
            response.message = "Plan executed successfully" if plan_success else "Plan execution failed"
        
        except Exception as e:
            response.success = False
            response.message = f"Error executing plan: {str(e)}"
            self.get_logger().error(f'Error in execute_plan_callback: {str(e)}')
        
        return response
    
    def get_robot_capabilities(self) -> dict:
        """Get robot capabilities for LLM planning"""
        # In practice, this would interface with robot description
        return [
            "navigate_to_object",
            "navigate_to_location", 
            "grasp_object",
            "release_object",
            "detect_object",
            "orient_to_object",
            "wait",
            "find_placement_surface",
            "open_gripper",
            "close_gripper"
        ]

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Error Handling and Recovery

### Planning Error Classification and Recovery

LLMs can generate plans that encounter runtime errors. A good system should be able to classify these errors and attempt recovery:

```python title="Error Handling and Recovery"
from enum import Enum

class PlanExecutionStatus(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    PARTIAL = "partial_success"
    RECOVERABLE_ERROR = "recoverable_error"

class ExecutionError(Exception):
    """Custom exception for planning execution errors"""
    def __init__(self, message, error_type, recoverable=True):
        super().__init__(message)
        self.error_type = error_type
        self.recoverable = recoverable

class PlanRecoverySystem:
    def __init__(self):
        self.recovery_strategies = {
            'object_not_found': self.recover_from_object_not_found,
            'navigation_failed': self.recover_from_navigation_failed,
            'grasp_failed': self.recover_from_grasp_failed,
            'collision_detected': self.recover_from_collision
        }
    
    def attempt_recovery(self, error: ExecutionError, original_plan: List[Dict], current_step: int) -> (PlanExecutionStatus, List[Dict]):
        """
        Attempt to recover from an execution error
        """
        if not error.recoverable:
            return PlanExecutionStatus.FAILURE, []
        
        recovery_func = self.recovery_strategies.get(error.error_type)
        if not recovery_func:
            return PlanExecutionStatus.FAILURE, []
        
        try:
            # Attempt recovery
            recovery_plan = recovery_func(error, original_plan, current_step)
            return PlanExecutionStatus.RECOVERABLE_ERROR, recovery_plan
        except Exception as e:
            self.get_logger().error(f"Recovery failed: {str(e)}")
            return PlanExecutionStatus.FAILURE, []
    
    def recover_from_object_not_found(self, error, original_plan, current_step):
        """Recovery when expected object is not found"""
        # Could implement search behavior, ask for clarification, or skip step
        recovery_plan = []
        
        # For example, add search behavior before the failed action
        obj_name = error.args[1] if len(error.args) > 1 else "unknown"
        recovery_plan.append({
            'step': current_step,
            'action': 'search_for_object',
            'parameters': {'object': obj_name},
            'description': f'Search for {obj_name}'
        })
        
        # Then continue with original plan
        recovery_plan.extend(original_plan[current_step:])
        return recovery_plan
    
    def recover_from_navigation_failed(self, error, original_plan, current_step):
        """Recovery when navigation fails"""
        # Could implement alternative pathfinding, obstacle avoidance, or request help
        recovery_plan = []
        
        # Try alternative navigation approach
        recovery_plan.append({
            'step': current_step,
            'action': 'navigate_with_avoidance',
            'parameters': original_plan[current_step].get('parameters', {}),
            'description': 'Navigate with obstacle avoidance'
        })
        
        # Continue with remaining steps
        recovery_plan.extend(original_plan[current_step+1:])
        return recovery_plan
    
    def recover_from_grasp_failed(self, error, original_plan, current_step):
        """Recovery when grasp fails"""
        recovery_plan = []
        
        # Try alternative grasp
        obj_name = error.args[1] if len(error.args) > 1 else "unknown"
        recovery_plan.append({
            'step': current_step,
            'action': 'grasp_with_adjustment',
            'parameters': {'object': obj_name, 'approach': 'side'},
            'description': f'Attempt side grasp on {obj_name}'
        })
        
        # Continue with remaining steps
        recovery_plan.extend(original_plan[current_step+1:])
        return recovery_plan
    
    def recover_from_collision(self, error, original_plan, current_step):
        """Recovery when collision is detected"""
        recovery_plan = []
        
        # Stop current action and replan
        recovery_plan.append({
            'step': current_step,
            'action': 'stop_execution',
            'parameters': {},
            'description': 'Stop due to collision'
        })
        
        recovery_plan.append({
            'step': current_step + 1,
            'action': 'replan_path',
            'parameters': original_plan[current_step].get('parameters', {}),
            'description': 'Replan with collision avoidance'
        })
        
        # Resume plan after replanning
        recovery_plan.extend(original_plan[current_step:])
        return recovery_plan

# Example usage in a complete planning and execution cycle
class RobustLLMPlanner:
    def __init__(self, llm_api_key: str):
        self.llm_planner = LLMBasedRobotPlanner(llm_api_key)
        self.recovery_system = PlanRecoverySystem()
    
    def execute_command_with_recovery(self, command: str) -> bool:
        """
        Execute command with error handling and recovery
        """
        original_plan = self.llm_planner.llm_bot.generate_plan(
            command=command,
            robot_capabilities=self.llm_planner.action_executor.capabilities,
            environment_info=self.llm_planner.action_executor.environment_objects
        )
        
        plan = original_plan.get('plan', [])
        current_step = 0
        
        while current_step < len(plan):
            step_info = plan[current_step]
            print(f"Executing step {current_step + 1}: {step_info['description']}")
            
            try:
                success = self.llm_planner.action_executor.execute_action(step_info)
                
                if not success:
                    raise ExecutionError(
                        f"Action failed: {step_info['action']}", 
                        'execution_failed', 
                        recoverable=True
                    )
                
                current_step += 1  # Move to next step
                
            except ExecutionError as e:
                print(f"Execution error at step {current_step + 1}: {str(e)}")
                
                # Attempt recovery
                status, recovery_plan = self.recovery_system.attempt_recovery(
                    e, plan, current_step
                )
                
                if status == PlanExecutionStatus.RECOVERABLE_ERROR:
                    print("Attempting recovery...")
                    plan = recovery_plan
                    # Don't increment current_step to retry with recovery plan
                else:
                    print("Recovery failed, stopping execution")
                    return False
        
        print("Command executed successfully with error recovery")
        return True

# Example usage
# robust_planner = RobustLLMPlanner("your-openai-api-key")
# success = robust_planner.execute_command_with_recovery("Pick up the blue cup and place it on the table")
```

## Hands-On Exercise

1. Set up an OpenAI API key and integrate with GPT-4 for task planning
2. Implement a simple robot command execution system
3. Create a world state model that tracks objects and robot capabilities
4. Implement validation checks for safety and feasibility
5. Add error handling and recovery mechanisms to handle execution failures
6. Test the system with various natural language commands

## Summary

LLM-based cognitive planning provides powerful capabilities for translating natural language commands into robotic actions. Success requires careful integration of world modeling, plan validation, safety checks, and error recovery. The combination of LLMs for high-level reasoning and traditional robotics systems for low-level execution creates flexible, robust robotic systems capable of understanding and acting on complex natural language commands. Proper validation and recovery mechanisms are essential to ensure safe operation in real-world environments.