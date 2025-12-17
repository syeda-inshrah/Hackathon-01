---
title: "Week 3 - Manipulation and Grasping with Humanoid Hands"
sidebar_label: "Week 3: Manipulation & Grasping"
description: "Understanding manipulation and grasping strategies for humanoid robots"
keywords: [manipulation, grasping, humanoid, hand, robotics, kinematics, control]
---

# Week 3: Manipulation and Grasping with Humanoid Hands

## Learning Outcomes

By the end of this week, you will be able to:
- Understand the anatomy and kinematics of humanoid hands
- Design grasp strategies for different object types
- Implement manipulation control algorithms
- Plan dexterous manipulation motions
- Integrate perception with manipulation planning
- Execute coordinated arm-hand movements

## Introduction to Humanoid Manipulation

Humanoid manipulation refers to the ability of humanoid robots to interact with objects in their environment using anthropomorphic hands and arms. This capability is essential for performing tasks in human environments where objects are designed for human manipulation.

### Key Challenges in Humanoid Manipulation

- **Dexterity**: Achieving human-like fine manipulation skills
- **Coordination**: Coordinating multiple joints for complex tasks
- **Grasp Planning**: Determining stable and functional grasps
- **Force Control**: Applying appropriate forces for object handling
- **Real-time Response**: Reacting to changing environmental conditions

## Humanoid Hand Anatomy and Kinematics

### Hand Structure

A typical humanoid hand consists of:
- **Palm**: Central structure with 5 digits
- **Digits**: 4 fingers + 1 thumb, each with multiple joints
- **Joints**: Usually 3 joints per finger (MCP, PIP, DIP)
- **Actuators**: Motors or tendons for joint actuation
- **Sensors**: Tactile, force, and position sensors

### Kinematic Considerations

Humanoid hands have high degrees of freedom, making the kinematic problem complex:

- **Forward Kinematics**: Determine fingertip positions from joint angles
- **Inverse Kinematics**: Determine joint angles for desired fingertip positions
- **Redundancy**: Multiple configurations can achieve the same grasp

```python title="Humanoid Hand Kinematics Example"
import numpy as np
import math

class HumanoidHandKinematics:
    def __init__(self):
        # Simplified finger model with 3 joints
        # Link lengths: MCP, PIP, DIP
        self.finger_lengths = {
            'thumb': [0.02, 0.03, 0.02],
            'index': [0.03, 0.04, 0.03],
            'middle': [0.03, 0.045, 0.03],
            'ring': [0.03, 0.04, 0.03],
            'pinky': [0.025, 0.035, 0.025]
        }
    
    def dh_transform(self, a, alpha, d, theta):
        """Calculate Denavit-Hartenberg transformation matrix"""
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def finger_forward_kinematics(self, finger_name, joint_angles):
        """Calculate forward kinematics for a single finger"""
        if finger_name not in self.finger_lengths:
            raise ValueError(f"Unknown finger: {finger_name}")
        
        lengths = self.finger_lengths[finger_name]
        if len(joint_angles) != len(lengths):
            raise ValueError(f"Expected {len(lengths)} joint angles for {finger_name}")
        
        # Start with identity transformation
        T = np.eye(4)
        
        for i, (length, angle) in enumerate(zip(lengths, joint_angles)):
            # For a finger, we can model as simple revolute joints
            # DH parameters for a simple finger: a=length, alpha=0, d=0, theta=angle
            A = self.dh_transform(length, 0, 0, angle)
            T = np.dot(T, A)
        
        # Return the position of the fingertip
        return T[:3, 3]
    
    def hand_forward_kinematics(self, hand_poses):
        """Calculate forward kinematics for the entire hand"""
        fingertip_positions = {}
        
        for finger_name, joint_angles in hand_poses.items():
            if finger_name in self.finger_lengths:
                pos = self.finger_forward_kinematics(finger_name, joint_angles)
                fingertip_positions[finger_name] = pos
        
        return fingertip_positions

# Example usage
hand_kin = HumanoidHandKinematics()

# Define a simple hand pose (all joints at 0.2 radians)
hand_poses = {
    'thumb': [0.2, 0.3, 0.2],
    'index': [0.2, 0.2, 0.2],
    'middle': [0.2, 0.2, 0.2],
    'ring': [0.2, 0.2, 0.2],
    'pinky': [0.2, 0.2, 0.2]
}

fingertip_positions = hand_kin.hand_forward_kinematics(hand_poses)
for finger, pos in fingertip_positions.items():
    print(f"{finger} fingertip: {pos}")
```

## Grasp Types and Planning

### Common Grasp Types

1. **Power Grasps**: For applying large forces (cylindrical, spherical, hook)
2. **Precision Grasps**: For fine manipulation (tip pinch, lateral, circular)

### Grasp Stability Metrics

- **Grasp Wrench Space**: Set of forces and torques the grasp can resist
- **Force Closure**: Ability to maintain grasp under any direction of force
- **Form Closure**: Stability through geometric constraints

```python title="Grasp Evaluation"
import numpy as np

class GraspEvaluator:
    def __init__(self):
        self.friction_coeff = 0.8  # Typical friction coefficient for rubber fingertips
    
    def compute_contact_jacobian(self, contact_points, contact_normals):
        """Compute the contact Jacobian matrix"""
        # For each contact point, we have 6 constraints (3 forces, 3 torques)
        # This is a simplified version for point contact with friction cone
        J = []
        
        for i, (point, normal) in enumerate(zip(contact_points, contact_normals)):
            # Create contact Jacobian for this point
            # Forces along normal direction
            force_normal = list(normal) + [0, 0, 0]  # Force component
            J.append(force_normal)
            
            # Forces tangential to normal (friction cone approximation)
            # Find two vectors perpendicular to normal
            if abs(normal[2]) < 0.9:  # Check if normal is not close to z-axis
                t1 = np.cross(normal, [0, 0, 1])
            else:
                t1 = np.cross(normal, [1, 0, 0])
            t1 = t1 / np.linalg.norm(t1)
            t2 = np.cross(normal, t1)
            
            # Tangential forces (friction)
            force_t1 = list(t1) + list(np.cross(point, t1))
            force_t2 = list(t2) + list(np.cross(point, t2))
            
            J.append(force_t1)
            J.append(force_t2)
        
        return np.array(J)
    
    def evaluate_grasp_stability(self, contact_points, contact_normals, object_weight=1.0):
        """Evaluate the stability of a grasp"""
        # Compute contact Jacobian
        J = self.compute_contact_jacobian(contact_points, contact_normals)
        
        # The grasp is stable if the grasp wrench space contains the negative of the external wrench
        # For simplicity, we only consider gravitational force
        external_wrench = np.array([0, 0, object_weight, 0, 0, 0])  # Gravity force + no torque
        
        # Check if external wrench is in the grasp wrench space
        # This is a simplified approach - full implementation would use polytope methods
        try:
            # Solve J^T * f = -external_wrench for contact forces
            # This requires more complex methods in practice
            # The following is a simplified stability metric
            J_pinv = np.linalg.pinv(J)
            forces = -np.dot(J_pinv, external_wrench)
            
            # Check if all forces are in friction cones
            is_stable = all(f >= 0 for f in forces[:len(forces)//2])  # Only normal forces for this example
            
            # Calculate condition number as stability metric
            stability_metric = 1.0 / (np.linalg.cond(J) + 1e-6)  # Higher is more stable
            
            return is_stable, stability_metric
        except:
            # If we can't solve, return basic stability estimate
            return False, 0.0

# Example: Evaluate a simple grasp
evaluator = GraspEvaluator()

# Define contact points and normals (simplified 2-finger grasp)
contact_points = [
    np.array([0.05, 0.0, 0.1]),   # Contact point 1 (right finger)
    np.array([0.05, 0.0, -0.1])   # Contact point 2 (left finger) - holding a cylinder
]

contact_normals = [
    np.array([-1.0, 0.0, 0.0]),   # Normal pointing inward from right finger
    np.array([1.0, 0.0, 0.0])     # Normal pointing inward from left finger
]

is_stable, stability_metric = evaluator.evaluate_grasp_stability(contact_points, contact_normals, object_weight=0.5)
print(f"Grasp stability: {is_stable}, Stability metric: {stability_metric:.3f}")
```

## Manipulation Control Strategies

### Impedance Control

Impedance control allows the robotic hand to behave like a virtual spring-damper system, making it compliant and suitable for interaction with objects.

```python title="Impedance Control for Manipulation"
import numpy as np

class ImpedanceController:
    def __init__(self, stiffness=1000, damping=20, mass=1.0):
        self.stiffness = np.array(stiffness) if isinstance(stiffness, (list, tuple)) else np.full(6, stiffness)
        self.damping = np.array(damping) if isinstance(damping, (list, tuple)) else np.full(6, damping)
        self.mass = np.array(mass) if isinstance(mass, (list, tuple)) else np.full(6, mass)
        
        # Initialize state
        self.target_pose = np.zeros(6)  # [x, y, z, rx, ry, rz]
        self.current_pose = np.zeros(6)
        self.current_vel = np.zeros(6)
    
    def update_target(self, target_pose):
        """Update the desired target pose"""
        self.target_pose = target_pose
    
    def compute_impedance_force(self, current_pose, current_vel, dt=0.01):
        """Compute the impedance control force/torque"""
        # Calculate position and velocity errors
        pos_error = current_pose[:3] - self.target_pose[:3]
        ori_error = current_pose[3:] - self.target_pose[3:]
        error = np.concatenate([pos_error, ori_error])
        
        vel_error = current_vel
        
        # Apply impedance control law: M*ẍ + D*ẋ + K*x = F
        # Rearranged: F = K*x + D*ẋ
        force = self.stiffness * error + self.damping * vel_error
        
        return force

# Example: Use impedance control for compliant manipulation
impedance_ctrl = ImpedanceController(
    stiffness=[1000, 1000, 1000, 100, 100, 50],  # Higher stiffness in position, lower in orientation
    damping=[200, 200, 200, 40, 40, 20]
)

# Set a target pose (position and orientation)
target_pose = np.array([0.5, 0.0, 0.3, 0.0, 0.0, 0.0])  # Position: 0.5, 0.0, 0.3
impedance_ctrl.update_target(target_pose)

# Simulate current state
current_pose = np.array([0.49, 0.01, 0.28, 0.01, -0.01, 0.02])
current_vel = np.array([0.01, -0.01, 0.02, 0.01, 0.0, -0.01])

# Compute impedance force
force = impedance_ctrl.compute_impedance_force(current_pose, current_vel)
print(f"Impedance control force/torque: {force}")
```

## Grasp Planning with Perception

### Integration with Vision Systems

Grasp planning requires perception of objects in the environment:

```python title="Perception-Guided Grasp Planning"
import numpy as np
from scipy.spatial.transform import Rotation as R

class PerceptionGuidedGraspPlanner:
    def __init__(self):
        self.object_database = {
            'cylinder': {
                'shape_params': {'radius': 0.025, 'height': 0.1},
                'preferred_grasps': ['cylindrical', 'tip_pinch']
            },
            'box': {
                'shape_params': {'width': 0.05, 'height': 0.03, 'depth': 0.08},
                'preferred_grasps': ['lateral', 'top_pinch']
            },
            'ball': {
                'shape_params': {'radius': 0.03},
                'preferred_grasps': ['spherical', 'tripod']
            }
        }
    
    def identify_object(self, point_cloud):
        """Identify object from point cloud data (simplified)"""
        # This would involve complex computer vision algorithms in practice
        # For this example, we'll return a simple classification
        # based on the dimensions of the bounding box
        
        # Simplified: look at the spread of points to identify shape
        if len(point_cloud) == 0:
            return None, {}
        
        # Calculate bounding box
        mins = np.min(point_cloud, axis=0)
        maxs = np.max(point_cloud, axis=0)
        dimensions = maxs - mins
        center = (mins + maxs) / 2
        
        # Identify based on aspect ratios
        sorted_dims = np.sort(dimensions)
        aspect_ratios = sorted_dims / sorted_dims[0]
        
        if aspect_ratios[1] < 1.5 and aspect_ratios[2] < 1.5:
            # Similar dimensions in all directions - likely a ball
            shape = 'ball'
            radius = sorted_dims[0] / 2
            params = {'radius': radius}
        elif aspect_ratios[2] > 3 and aspect_ratios[1] < 2:
            # One dimension much larger - likely a cylinder
            shape = 'cylinder'
            radius = min(dimensions[0], dimensions[1]) / 2
            height = max(dimensions)
            params = {'radius': radius, 'height': height}
        else:
            # Other - likely a box
            shape = 'box'
            params = {
                'width': dimensions[0],
                'height': dimensions[1], 
                'depth': dimensions[2]
            }
        
        return shape, {'center': center, 'dimensions': dimensions, 'params': params}
    
    def plan_grasps(self, object_shape, object_params):
        """Plan appropriate grasps for the identified object"""
        if object_shape not in self.object_database:
            print(f"Unknown object shape: {object_shape}")
            return []
        
        obj_info = self.object_database[object_shape]
        preferred_grasps = obj_info['preferred_grasps']
        
        grasps = []
        
        for grasp_type in preferred_grasps:
            grasp_poses = self.generate_grasp_poses_for_shape(
                object_shape, object_params, grasp_type
            )
            for pose in grasp_poses:
                grasps.append({
                    'type': grasp_type,
                    'pose': pose,
                    'quality': self.estimate_grasp_quality(pose, object_shape, object_params)
                })
        
        return grasps
    
    def generate_grasp_poses_for_shape(self, shape, params, grasp_type):
        """Generate candidate grasp poses for a given shape and grasp type"""
        grasps = []
        
        if shape == 'cylinder' and grasp_type == 'cylindrical':
            # Opposing grasp on either side of cylinder
            center = params['center']
            radius = params['params']['radius']
            height = params['params']['height']
            
            # Grasp along x-axis
            grasp1 = np.array([
                center[0], center[1] - radius - 0.02, center[2],  # Position
                0, 0, 0, 1  # Quaternion (approaching from y-direction)
            ])
            grasp2 = np.array([
                center[0], center[1] + radius + 0.02, center[2],  # Position
                0, 0, 0, 1  # Quaternion
            ])
            
            grasps.extend([grasp1, grasp2])
        
        elif shape == 'box' and grasp_type == 'top_pinch':
            # Grasp from top of box
            center = params['center']
            width = params['params']['width']
            height = params['params']['height']
            depth = params['params']['depth']
            
            # Approach from above
            grasp1 = np.array([
                center[0] - width/2 - 0.01, center[1], center[2] + height/2 + 0.02,  # Position
                0.707, 0, 0.707, 0  # Quaternion (rotated 90 deg around x)
            ])
            grasp2 = np.array([
                center[0] + width/2 + 0.01, center[1], center[2] + height/2 + 0.02,  # Position
                0.707, 0, 0.707, 0  # Quaternion
            ])
            
            grasps.extend([grasp1, grasp2])
        
        # Add more grasp types and shapes as needed
        
        return grasps
    
    def estimate_grasp_quality(self, grasp_pose, object_shape, object_params):
        """Estimate the quality of a grasp (simplified)"""
        # This would involve complex physics simulations in practice
        # For this example, we'll use a simple heuristic
        
        # Factors to consider: 
        # - Distance from object center
        # - Orientation relative to object
        # - Access to grasp point
        
        quality = 0.8  # Base quality
        
        # Add object-specific adjustments
        if object_shape == 'cylinder':
            # Penalize grasps that are too close to the top/bottom
            height_ratio = abs(grasp_pose[2] - object_params['center'][2]) / (object_params['params']['height']/2)
            if height_ratio > 0.8:
                quality -= 0.2
        
        return min(1.0, max(0.0, quality))

# Example usage
planner = PerceptionGuidedGraspPlanner()

# Simulate point cloud for a cylindrical object
cylinder_points = []
for i in range(100):
    angle = np.random.uniform(0, 2*np.pi)
    height = np.random.uniform(-0.05, 0.05)
    r = np.random.uniform(0.024, 0.026)  # radius
    x = r * np.cos(angle)
    y = r * np.sin(angle) 
    z = height
    cylinder_points.append([x, y, z])

cylinder_points = np.array(cylinder_points)

# Identify the object
obj_shape, obj_params = planner.identify_object(cylinder_points)
print(f"Identified object: {obj_shape}")
print(f"Object params: {obj_params}")

# Plan grasps
grasps = planner.plan_grasps(obj_shape, obj_params)
print(f"Generated {len(grasps)} potential grasps")

for i, grasp in enumerate(grasps[:3]):  # Show first 3 grasps
    print(f"Grasp {i+1}: {grasp['type']}, quality: {grasp['quality']:.2f}")
```

## ROS 2 Implementation for Manipulation

### Coordination of Arm and Hand

```python title="ROS 2 Manipulation Controller"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidManipulationController(Node):
    def __init__(self):
        super().__init__('humanoid_manipulation_controller')
        
        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create publishers
        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_joint_commands',
            10
        )
        
        self.hand_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/hand_joint_commands',
            10
        )
        
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/manipulation_target',
            self.target_pose_callback,
            10
        )
        
        # Initialize joint positions
        self.joint_positions = {}
        self.joint_velocities = {}
        
        # Initialize manipulation controller
        self.arm_ik_solver = self.initialize_arm_ik()
        self.current_target = None
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz control
        
        self.get_logger().info('Humanoid Manipulation Controller initialized')
    
    def initialize_arm_ik(self):
        """Initialize arm inverse kinematics solver (placeholder)"""
        # In a real implementation, this would be a full IK solver
        # like KDL, Trac-IK, or custom implementation
        return "IK_SOLVER_INITIALIZED"
    
    def joint_state_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
    
    def target_pose_callback(self, msg):
        """Update manipulation target"""
        self.current_target = msg.pose
        self.get_logger().info(f"New manipulation target received: {self.current_target}")
    
    def control_loop(self):
        """Main manipulation control loop"""
        if self.current_target is None:
            return
        
        # Solve inverse kinematics for arm
        joint_goals = self.solve_arm_ik(self.current_target)
        if joint_goals is not None:
            # Publish arm commands
            arm_cmd_msg = Float64MultiArray()
            arm_cmd_msg.data = joint_goals[:7]  # First 7 joints for arm
            self.arm_cmd_pub.publish(arm_cmd_msg)
            
            # Later, when grasp is needed, publish hand commands
            hand_cmd_msg = Float64MultiArray()
            # For now, publish open hand position (all fingers extended)
            hand_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 10 finger joints
            self.hand_cmd_pub.publish(hand_cmd_msg)
    
    def solve_arm_ik(self, target_pose):
        """Solve inverse kinematics for the arm (simplified)"""
        # In a real implementation, this would use a proper IK solver
        # For now, return placeholder values
        return [0.1, 0.2, -0.3, 0.4, 0.1, -0.2, 0.3] + [0.0] * 10  # Arm + hand joints

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidManipulationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Dexterity and Complex Manipulation

### Multi-finger Coordination

Complex manipulation often requires coordinated movement of multiple fingers. This can be achieved through:

1. **Predefined Synergies**: Common patterns of finger movement
2. **Task-based Grasp Planning**: Grasps optimized for specific tasks
3. **Learning from Demonstration**: Imitating human manipulation

## Hands-On Exercise

1. Implement a grasp evaluator that can assess the stability of different grasp configurations
2. Create a simple manipulation controller that can reach for a target position
3. Integrate perception to identify objects and plan appropriate grasps
4. Test your manipulation system in Gazebo with different object shapes
5. Implement a simple task like picking up a cup and moving it to a new location

## Summary

Humanoid manipulation combines complex kinematics, grasp planning, and control theory to enable robots to interact with objects in human environments. Success requires understanding of hand anatomy, grasp mechanics, and the integration of perception with action. The dexterity of human-like hands provides the potential for versatile manipulation, but realizing this potential requires sophisticated control algorithms and coordinated whole-arm motion planning. The integration of perception with manipulation planning enables robots to adapt their manipulation strategies based on object properties and environmental conditions.