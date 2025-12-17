---
title: "Week 1 - Kinematics and Dynamics of Humanoid Robots"
sidebar_label: "Week 1: Kinematics & Dynamics"
description: "Understanding the kinematic and dynamic models for humanoid robots"
keywords: [kinematics, dynamics, humanoid, robot, forward, inverse, jacobian, control]
---

# Week 1: Kinematics and Dynamics of Humanoid Robots

## Learning Outcomes

By the end of this week, you will be able to:
- Understand the kinematic structure of humanoid robots
- Calculate forward and inverse kinematics for humanoid limbs
- Apply Jacobian matrices to motion control
- Model the dynamic properties of humanoid systems
- Design control strategies based on dynamic models

## Introduction to Humanoid Kinematics

Humanoid robots have anthropomorphic kinematic structures that mimic the human form. Understanding these structures is essential for controlling movement, maintaining balance, and performing manipulation tasks.

### Key Characteristics of Humanoid Kinematics

- **Articulated Structure**: Multiple interconnected rigid bodies
- **Degrees of Freedom**: Typically 20-40+ DOF for full humanoid systems
- **Redundancy**: More DOF than required for specific tasks
- **Anthropomorphic Design**: Limbs arranged similarly to human body

### Humanoid Robot Architecture

A typical humanoid robot consists of:

- **Torso**: Central body with head, arms, and pelvis attachments
- **Arms**: Shoulders, elbows, wrists, and hands
- **Legs**: Hips, knees, ankles, and feet
- **Sensors**: IMUs, cameras, force/torque sensors
- **Actuators**: Motors at each joint for movement control

## Kinematic Modeling

### Forward Kinematics

Forward kinematics determines the end-effector position and orientation given joint angles. This is straightforward to compute but essential for understanding robot state.

For a humanoid arm with joint angles θ₁, θ₂, ..., θₙ:

```
T = A₁(θ₁) × A₂(θ₂) × ... × Aₙ(θₙ)
```

Where T is the transformation matrix representing the end-effector pose, and Aᵢ(θᵢ) are the transformation matrices for each joint.

```python title="Forward Kinematics Example for Humanoid Arm"
import numpy as np
import math

def dh_transform(a, alpha, d, theta):
    """Calculate Denavit-Hartenberg transformation matrix"""
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_humanoid_arm(joint_angles, dh_params):
    """
    Calculate forward kinematics for a humanoid arm
    
    Args:
        joint_angles: List of joint angles [θ1, θ2, ..., θn]
        dh_params: List of DH parameters [(a, alpha, d, θ_offset), ...]
    
    Returns:
        Transformation matrix representing end-effector pose
    """
    T = np.eye(4)  # Start with identity matrix
    
    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset
        A = dh_transform(a, alpha, d, theta)
        T = np.dot(T, A)
    
    return T

# Example: Simple 3-DOF planar arm
# DH parameters: (a, alpha, d, theta_offset)
dh_params = [
    (0.1, math.pi/2, 0.0, 0.0),    # Shoulder joint
    (0.2, 0.0, 0.0, 0.0),          # Elbow joint
    (0.15, 0.0, 0.0, 0.0)          # Wrist joint
]

# Example joint angles in radians
joint_angles = [0.1, 0.2, 0.3]

end_effector_pose = forward_kinematics_humanoid_arm(joint_angles, dh_params)
print("End-effector pose:")
print(end_effector_pose)
```

### Inverse Kinematics

Inverse kinematics determines the joint angles required to achieve a desired end-effector position and orientation. This is more complex than forward kinematics and often has multiple solutions or no solution.

#### Analytical vs Numerical Solutions

- **Analytical**: Exact solution, but only exists for specific kinematic chains
- **Numerical**: Approximate solution using iterative methods, applicable to general chains

```python title="Inverse Kinematics using Jacobian Transpose Method"
import numpy as np

class HumanoidIKSolver:
    def __init__(self, dh_params, max_iterations=100, tolerance=1e-4):
        self.dh_params = dh_params
        self.max_iterations = max_iterations
        self.tolerance = tolerance
    
    def jacobian(self, joint_angles):
        """Calculate the Jacobian matrix for the robot"""
        n = len(joint_angles)
        J = np.zeros((6, n))  # 6 DOF (3 position + 3 orientation)
        
        # Forward kinematics to get all joint positions and orientations
        T = np.eye(4)
        joint_positions = [T[:3, 3]]  # Store position of each joint
        joint_orientations = [T[:3, :3]]  # Store orientation of each joint
        
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            A = self.dh_transform(a, alpha, d, theta)
            T = np.dot(T, A)
            joint_positions.append(T[:3, 3])
            joint_orientations.append(T[:3, :3])
        
        # Calculate Jacobian
        end_effector_pos = joint_positions[-1]
        
        for i in range(n):
            # Z-axis of joint i
            z_i = joint_orientations[i][:3, 2]
            # Position of end effector relative to joint i
            p_i = end_effector_pos - joint_positions[i]
            
            # Linear velocity component
            J[:3, i] = np.cross(z_i, p_i)
            # Angular velocity component
            J[3:, i] = z_i
        
        return J
    
    def inverse_kinematics(self, target_pose, initial_angles):
        """Solve inverse kinematics using Jacobian transpose method"""
        current_angles = np.array(initial_angles)
        
        for iteration in range(self.max_iterations):
            # Calculate current end-effector pose
            current_pose = self.forward_kinematics(current_angles)
            current_pos = current_pose[:3, 3]
            target_pos = target_pose[:3, 3]
            
            # Calculate error
            pos_error = target_pos - current_pos
            total_error = np.linalg.norm(pos_error)
            
            if total_error < self.tolerance:
                print(f"Converged after {iteration} iterations")
                return current_angles
            
            # Calculate Jacobian
            J = self.jacobian(current_angles)
            
            # Use Jacobian transpose method for update
            angle_updates = np.dot(J[:3, :].T, pos_error)  # Only position for this example
            current_angles += 0.1 * angle_updates  # Learning rate
            
            # Check for joint limits (simplified)
            current_angles = np.clip(current_angles, -np.pi, np.pi)
        
        print(f"Did not converge after {self.max_iterations} iterations")
        return current_angles
    
    def dh_transform(self, a, alpha, d, theta):
        """Calculate Denavit-Hartenberg transformation matrix"""
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles):
        """Calculate forward kinematics"""
        T = np.eye(4)
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            A = self.dh_transform(a, alpha, d, theta)
            T = np.dot(T, A)
        return T

# Example usage
dh_params = [
    (0.1, math.pi/2, 0.0, 0.0),
    (0.2, 0.0, 0.0, 0.0),
    (0.15, 0.0, 0.0, 0.0)
]

ik_solver = HumanoidIKSolver(dh_params)

# Target pose (simplified - just position for example)
target_pose = np.eye(4)
target_pose[:3, 3] = [0.3, 0.1, 0.2]  # Target position

# Initial joint angles
initial_angles = [0.0, 0.0, 0.0]

# Solve inverse kinematics
result_angles = ik_solver.inverse_kinematics(target_pose, initial_angles)
print("Resulting joint angles:", result_angles)
```

## Dynamic Modeling

### Newton-Euler Formulation

The Newton-Euler formulation is used to model the dynamic behavior of robotic systems by considering the forces and moments acting on each link.

For each link i:
- Linear motion: Fᵢ = mᵢaᵢ
- Angular motion: Nᵢ = Ḣᵢ

Where Fᵢ is the force, mᵢ is the mass, aᵢ is the linear acceleration, Nᵢ is the moment, and Ḣᵢ is the rate of change of angular momentum.

### Lagrangian Formulation

The Lagrangian formulation uses energy methods to derive equations of motion:

L = T - V

Where L is the Lagrangian, T is the kinetic energy, and V is the potential energy.

The equations of motion are derived using the Euler-Lagrange equation:

d/dt(∂L/∂q̇ᵢ) - ∂L/∂qᵢ = Qᵢ

Where qᵢ are the generalized coordinates and Qᵢ are the generalized forces.

```python title="Dynamic Model for Simple Humanoid Leg"
import numpy as np
import math

class HumanoidLegDynamics:
    def __init__(self):
        # Robot parameters
        self.l1 = 0.3  # Upper leg length (m)
        self.l2 = 0.3  # Lower leg length (m)
        self.m1 = 5.0  # Upper leg mass (kg)
        self.m2 = 4.0  # Lower leg mass (kg)
        self.I1 = 0.2  # Upper leg moment of inertia
        self.I2 = 0.15  # Lower leg moment of inertia
        
    def compute_mass_matrix(self, q1, q2):
        """Compute the mass matrix for the leg system"""
        # Joint angles
        th1 = q1
        th2 = q2
        th12 = th1 + th2
        
        # Mass matrix elements
        h11 = (self.m1 + self.m2) * self.l1**2 + self.m2 * self.l2**2 + 2*self.m2*self.l1*self.l2*math.cos(th2) + self.I1 + self.I2
        h12 = self.m2 * self.l2**2 + self.m2*self.l1*self.l2*math.cos(th2) + self.I2
        h21 = h12
        h22 = self.m2 * self.l2**2 + self.I2
        
        H = np.array([[h11, h12], 
                      [h21, h22]])
        return H
    
    def compute_coriolis_gravity(self, q1, q2, dq1, dq2):
        """Compute Coriolis and gravity forces"""
        # Joint angles and velocities
        th1 = q1
        th2 = q2
        dth1 = dq1
        dth2 = dq2
        
        # Coriolis terms
        C1 = -self.m2*self.l1*self.l2*math.sin(th2) * (2*dth1*dth2 + dth2**2)
        C2 = self.m2*self.l1*self.l2*math.sin(th2) * (dth1**2)
        
        # Gravity terms
        g = 9.81  # gravity constant
        G1 = (self.m1*self.l1 + self.m2*self.l1) * g * math.cos(th1) + self.m2*self.l2 * g * math.cos(th1 + th2)
        G2 = self.m2*self.l2 * g * math.cos(th1 + th2)
        
        C = np.array([C1 + G1, C2 + G2])
        return C
    
    def compute_torques(self, q, dq, ddq):
        """Compute required torques for given motion"""
        q1, q2 = q
        dq1, dq2 = dq
        ddq1, ddq2 = ddq
        
        # Compute mass matrix
        H = self.compute_mass_matrix(q1, q2)
        
        # Compute Coriolis and gravity forces
        C = self.compute_coriolis_gravity(q1, q2, dq1, dq2)
        
        # Compute required torques: τ = H(q)q̈ + C(q, q̇)
        ddq_vec = np.array([ddq1, ddq2])
        torques = np.dot(H, ddq_vec) + C
        
        return torques

# Example usage
leg_dynamics = HumanoidLegDynamics()

# Define a trajectory
q = [0.1, 0.2]  # joint angles
dq = [0.5, 0.3]  # joint velocities
ddq = [2.0, 1.0]  # joint accelerations

torques = leg_dynamics.compute_torques(q, dq, ddq)
print(f"Required torques: {torques}")
```

## Center of Mass and Stability

### Center of Mass Calculation

For stable locomotion, it's crucial to understand the center of mass (CoM) of the humanoid system.

For a system of n links:
```
CoM = Σ(mᵢ * rᵢ) / Σ(mᵢ)
```

Where mᵢ is the mass of link i and rᵢ is the position vector of link i's center of mass.

### Zero Moment Point (ZMP)

The Zero Moment Point is critical for bipedal stability:
```
ZMP_x = (Σ(mᵢ*g*zᵢ*xᵢ - mᵢ*zᵢ*ẍᵢ) / Σ(mᵢ*g*zᵢ - mᵢ*z̈ᵢ)
```

Where xᵢ is the x-coordinate of the center of mass of the i-th element.

```python title="Center of Mass and ZMP Calculation"
import numpy as np

class HumanoidStabilityAnalyzer:
    def __init__(self):
        # Simplified humanoid model with point masses
        self.link_data = [
            {'name': 'head', 'mass': 5.0, 'position': np.array([0, 0, 1.7])},
            {'name': 'torso', 'mass': 20.0, 'position': np.array([0, 0, 1.2])},
            {'name': 'left_arm', 'mass': 3.0, 'position': np.array([-0.3, 0, 1.3])},
            {'name': 'right_arm', 'mass': 3.0, 'position': np.array([0.3, 0, 1.3])},
            {'name': 'left_leg', 'mass': 6.0, 'position': np.array([-0.1, 0, 0.7])},
            {'name': 'right_leg', 'mass': 6.0, 'position': np.array([0.1, 0, 0.7])},
            {'name': 'feet', 'mass': 2.0, 'position': np.array([0, 0, 0.1])}
        ]
    
    def calculate_com(self, positions=None):
        """Calculate center of mass of the humanoid"""
        if positions is not None:
            # Update positions if provided
            for i in range(len(self.link_data)):
                self.link_data[i]['position'] = positions[i]
        
        total_mass = 0
        weighted_sum = np.array([0.0, 0.0, 0.0])
        
        for link in self.link_data:
            total_mass += link['mass']
            weighted_sum += link['mass'] * link['position']
        
        com = weighted_sum / total_mass
        return com
    
    def is_zmp_stable(self, com_pos, com_acc, foot_positions, threshold=0.1):
        """Check if the ZMP is within the support polygon"""
        # Simplified ZMP calculation
        g = 9.81  # gravity
        
        # ZMP in x-direction
        zmp_x = com_pos[0] - (com_acc[2] * (com_pos[2] - 0.0)) / (g + com_acc[2])
        # ZMP in y-direction  
        zmp_y = com_pos[1] - (com_acc[2] * (com_pos[2] - 0.0)) / (g + com_acc[2])
        
        # Define support polygon (simplified as rectangle between feet)
        min_x = min(foot[0] for foot in foot_positions)
        max_x = max(foot[0] for foot in foot_positions)
        min_y = min(foot[1] for foot in foot_positions)
        max_y = max(foot[1] for foot in foot_positions)
        
        # Check if ZMP is within support polygon
        is_stable = (min_x - threshold <= zmp_x <= max_x + threshold and 
                     min_y - threshold <= zmp_y <= max_y + threshold)
        
        return is_stable, (zmp_x, zmp_y)

# Example usage
analyzer = HumanoidStabilityAnalyzer()

# Calculate current CoM
current_com = analyzer.calculate_com()
print(f"Current center of mass: {current_com}")

# Check stability
foot_positions = [np.array([-0.1, -0.1, 0.0]), np.array([0.1, 0.1, 0.0])]
com_pos = current_com
com_acc = np.array([0.0, 0.0, 0.0])  # No acceleration in this example

is_stable, zmp = analyzer.is_zmp_stable(com_pos, com_acc, foot_positions)
print(f"Is stable: {is_stable}, ZMP: {zmp}")
```

## ROS 2 Implementation

### Kinematics with KDL and MoveIt!

```python title="Kinematics Implementation using ROS 2"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener
import numpy as np

class HumanoidKinematicsNode(Node):
    def __init__(self):
        super().__init__('humanoid_kinematics_node')
        
        # Create subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create publisher for end-effector pose
        self.ee_pose_pub = self.create_publisher(
            Pose,
            '/end_effector_pose',
            10
        )
        
        # Initialize joint positions
        self.joint_positions = {}
        
        # For this example, we'll implement a simple FK calculation
        # In practice, you would use MoveIt! or KDL
        self.get_logger().info('Humanoid Kinematics node initialized')
    
    def joint_state_callback(self, msg):
        """Process joint state messages"""
        # Update joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
        
        # Calculate forward kinematics for right arm (example)
        ee_pose = self.calculate_right_arm_fk()
        
        # Publish end-effector pose
        self.ee_pose_pub.publish(ee_pose)
    
    def calculate_right_arm_fk(self):
        """Calculate forward kinematics for right arm - simplified example"""
        # In a real implementation, this would use the full kinematic chain
        # and DH parameters or use MoveIt!/KDL
        pose = Pose()
        
        # Placeholder calculation - in reality, this would be calculated
        # from joint angles using kinematic equations
        pose.position.x = 0.5
        pose.position.y = -0.3
        pose.position.z = 1.2
        
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidKinematicsNode()
    
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

## Hands-On Exercise

1. Create a URDF model of a simplified humanoid robot (torso, 2 arms, 2 legs)
2. Calculate the forward kinematics for one arm using the DH parameters
3. Implement an inverse kinematics solver for the same arm
4. Calculate the center of mass for different poses of your robot
5. Simulate the robot in Gazebo and verify your kinematic calculations

## Summary

Understanding the kinematics and dynamics of humanoid robots is fundamental to controlling their motion and ensuring stability. Forward kinematics allows us to determine end-effector positions from joint angles, while inverse kinematics solves the reverse problem. Dynamic modeling helps us understand the forces required for movement, and stability analysis ensures our humanoid robot can maintain balance during locomotion. These mathematical foundations are essential for all higher-level control and planning algorithms in humanoid robotics.