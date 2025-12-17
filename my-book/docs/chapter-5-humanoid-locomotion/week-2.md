---
title: "Week 2 - Bipedal Walking and Balance Control"
sidebar_label: "Week 2: Walking & Balance"
description: "Understanding bipedal locomotion and balance control for humanoid robots"
keywords: [bipedal, walking, balance, control, humanoid, zmp, locomotion, robotics]
---

# Week 2: Bipedal Walking and Balance Control

## Learning Outcomes

By the end of this week, you will be able to:
- Understand the principles of bipedal locomotion
- Implement walking pattern generators for humanoid robots
- Design balance control systems using feedback control
- Apply ZMP (Zero Moment Point) theory to walking control
- Simulate stable walking gaits for humanoid robots
- Analyze and tune walking controllers for stability

## Introduction to Bipedal Locomotion

Bipedal locomotion is one of the most challenging problems in robotics. Unlike wheeled robots or tracked vehicles, bipedal robots must manage complex dynamics with a continuously changing support point. The robot is essentially in constant motion between falling and catching itself.

### Key Challenges in Bipedal Walking

- **Dynamic Balance**: The robot is never statically stable
- **Underactuation**: Fewer actuators than degrees of freedom during flight phases
- **Impact Dynamics**: High forces during foot contacts
- **Real-time Control**: Require continuous balance adjustments
- **Terrain Adaptation**: Handle uneven surfaces and obstacles

### Walking Phases

Bipedal walking consists of two main phases:

1. **Single Support Phase**: One foot is on the ground, the other is swinging
2. **Double Support Phase**: Both feet are on the ground (momentarily)

## Zero Moment Point (ZMP) Theory

### Understanding ZMP

The Zero Moment Point (ZMP) is a crucial concept in bipedal robotics that helps determine the stability of a walking robot. It's the point on the ground where the moment of the ground reaction force equals zero.

Mathematically, the ZMP is defined as:
```
ZMP_x = (Σ(m_i * g * z_i * x_i - m_i * z_i * d²x_i/dt²)) / (Σ(m_i * g * z_i - m_i * d²z_i/dt²))
```

```
ZMP_y = (Σ(m_i * g * z_i * y_i - m_i * z_i * d²y_i/dt²)) / (Σ(m_i * g * z_i - m_i * d²z_i/dt²))
```

Where:
- (x_i, y_i, z_i) is the position of the i-th mass element
- m_i is the mass of the i-th element
- g is the gravitational acceleration
- The double dot represents the second time derivative (acceleration)

### ZMP Stability Criteria

For stable locomotion, the ZMP must remain within the support polygon (the polygon formed by the feet in contact with the ground).

## Walking Pattern Generation

### Preview Control Method

Preview control is a common approach for generating stable walking patterns based on ZMP feedback. This method uses future reference ZMP positions to calculate appropriate center of mass (CoM) trajectories.

```python title="ZMP-based Walking Pattern Generator"
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import math

class ZMPWalkingPatternGenerator:
    def __init__(self, com_height=0.8, sampling_period=0.01, preview_window=2.0):
        self.com_height = com_height  # CoM height in meters
        self.omega = math.sqrt(9.81 / com_height)  # Natural frequency
        self.dt = sampling_period
        self.preview_time = preview_window
        self.preview_steps = int(preview_window / sampling_period)
    
    def generate_foot_steps(self, step_length=0.3, step_width=0.2, n_steps=10):
        """Generate footstep sequence for walking"""
        footsteps = []
        
        # Starting position
        current_x = 0.0
        current_y = 0.0
        
        for i in range(n_steps):
            # Determine which foot (left or right)
            if i % 2 == 0:  # Right foot
                current_y = step_width / 2
                phase = "right"
            else:  # Left foot
                current_y = -step_width / 2
                phase = "left"
            
            # Move forward
            current_x += step_length
            
            footsteps.append({
                'x': current_x,
                'y': current_y,
                'phase': phase,
                'step_num': i
            })
        
        return footsteps
    
    def compute_reference_zmp(self, footsteps, double_support_time=0.1):
        """Compute reference ZMP trajectory based on footsteps"""
        n_steps = len(footsteps)
        total_time = n_steps * 1.0  # 1 second per step
        n_samples = int(total_time / self.dt)
        
        # Initialize ZMP trajectory
        ref_zmp_x = np.zeros(n_samples)
        ref_zmp_y = np.zeros(n_samples)
        
        # Calculate ZMP reference for each phase
        for i, footstep in enumerate(footsteps):
            start_idx = int(i / len(footsteps) * n_samples)
            end_idx = int((i + 1) / len(footsteps) * n_samples)
            
            if end_idx >= len(ref_zmp_x):
                end_idx = len(ref_zmp_x)
            
            # Set ZMP reference to foot position during support phase
            for idx in range(start_idx, end_idx):
                if idx < len(ref_zmp_x):
                    ref_zmp_x[idx] = footstep['x']
                    ref_zmp_y[idx] = footstep['y']
        
        return ref_zmp_x, ref_zmp_y
    
    def solve_com_trajectory(self, ref_zmp_x, ref_zmp_y):
        """Solve for CoM trajectory using preview control"""
        n_samples = len(ref_zmp_x)
        
        # Initialize CoM trajectory
        com_x = np.zeros(n_samples)
        com_y = np.zeros(n_samples)
        com_z = np.full(n_samples, self.com_height)
        
        # Initialize CoM velocity and acceleration
        com_dx = np.zeros(n_samples)
        com_dy = np.zeros(n_samples)
        
        com_ddx = np.zeros(n_samples)
        com_ddy = np.zeros(n_samples)
        
        # Simple implementation: CoM tracks ZMP with a fixed relationship
        # In a real implementation, proper preview control would be used
        for i in range(1, n_samples):
            # Simple PD control to keep ZMP near reference
            zmp_x_error = ref_zmp_x[i] - (com_x[i-1] - com_ddx[i-1] / (self.omega**2))
            zmp_y_error = ref_zmp_y[i] - (com_y[i-1] - com_ddy[i-1] / (self.omega**2))
            
            # Update CoM acceleration to correct ZMP error
            com_ddx[i] = self.omega**2 * (com_x[i-1] - ref_zmp_x[i] + zmp_x_error * 0.1)
            com_ddy[i] = self.omega**2 * (com_y[i-1] - ref_zmp_y[i] + zmp_y_error * 0.1)
            
            # Update velocity and position using simple integration
            com_dx[i] = com_dx[i-1] + com_ddx[i] * self.dt
            com_dy[i] = com_dy[i-1] + com_ddy[i] * self.dt
            
            com_x[i] = com_x[i-1] + com_dx[i] * self.dt + 0.5 * com_ddx[i] * self.dt**2
            com_y[i] = com_y[i-1] + com_dy[i] * self.dt + 0.5 * com_ddy[i] * self.dt**2
        
        return com_x, com_y, com_z, com_dx, com_dy

# Example usage
generator = ZMPWalkingPatternGenerator(com_height=0.8, sampling_period=0.01)
footsteps = generator.generate_foot_steps(step_length=0.3, step_width=0.2, n_steps=8)
ref_zmp_x, ref_zmp_y = generator.compute_reference_zmp(footsteps)
com_x, com_y, com_z, com_dx, com_dy = generator.solve_com_trajectory(ref_zmp_x, ref_zmp_y)

print(f"Generated {len(footsteps)} footsteps")
print(f"CoM trajectory has {len(com_x)} samples")
```

### Linear Inverted Pendulum Model (LIPM)

The Linear Inverted Pendulum Model is a simplified model of bipedal walking that assumes the center of mass moves at a constant height with varying horizontal position.

The LIPM equation is:
```
ẍ_com = ω²(x_com - x_zmp)
```

Where:
- ω² = g/h (g: gravity, h: height of CoM)
- x_com: CoM x-position
- x_zmp: ZMP x-position

## Balance Control Systems

### Feedback Controllers for Balance

Balance control typically involves multiple feedback loops:

1. **ZMP Feedback**: Corrects CoM position relative to ZMP
2. **Attitude Control**: Maintains trunk orientation
3. **Foot Leveling**: Ensures feet remain horizontal

```python title="Balance Control Implementation"
import numpy as np
import math

class BalanceController:
    def __init__(self, robot_mass=70.0, com_height=0.8, control_freq=100):
        self.mass = robot_mass
        self.com_height = com_height
        self.gravity = 9.81
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # Controller gains
        self.kp_zmp = 50.0   # Proportional gain for ZMP control
        self.kd_zmp = 10.0   # Derivative gain for ZMP control
        self.kp_att = 100.0  # Proportional gain for attitude control
        self.kd_att = 10.0   # Derivative gain for attitude control
        
        # Initialize state
        self.current_com_pos = np.array([0.0, 0.0, com_height])
        self.current_com_vel = np.array([0.0, 0.0, 0.0])
        self.current_attitude = np.array([0.0, 0.0, 0.0])  # Roll, pitch, yaw
        self.current_attitude_vel = np.array([0.0, 0.0, 0.0])
        
        # Reference values
        self.ref_zmp = np.array([0.0, 0.0])  # Reference ZMP position
        self.ref_attitude = np.array([0.0, 0.0, 0.0])  # Reference attitude
    
    def update_state(self, com_pos, com_vel, attitude, attitude_vel):
        """Update the current state of the robot"""
        self.current_com_pos = com_pos
        self.current_com_vel = com_vel
        self.current_attitude = attitude
        self.current_attitude_vel = attitude_vel
    
    def compute_balance_control(self):
        """Compute balance control torques based on current state"""
        # Calculate CoM ZMP error
        measured_zmp_x = self.current_com_pos[0] - (self.current_com_pos[2] / self.gravity) * self.current_com_vel[0]
        measured_zmp_y = self.current_com_pos[1] - (self.current_com_pos[2] / self.gravity) * self.current_com_vel[1]
        
        zmp_error = np.array([measured_zmp_x, measured_zmp_y]) - self.ref_zmp
        
        # Calculate attitude error
        attitude_error = self.current_attitude - self.ref_attitude
        
        # ZMP-based control (simplified - in practice, this drives joint torques)
        zmp_control = -self.kp_zmp * zmp_error - self.kd_zmp * self.current_com_vel[:2]
        
        # Attitude control
        attitude_control = -self.kp_att * attitude_error[:2] - self.kd_att * self.current_attitude_vel[:2]
        
        # Combine controls
        total_control = np.concatenate([zmp_control, attitude_control])
        
        return total_control

# Example usage
controller = BalanceController()

# Simulate robot state
com_pos = np.array([0.01, -0.01, 0.8])
com_vel = np.array([0.0, 0.0, 0.0])
attitude = np.array([0.01, -0.005, 0.0])  # Small roll and pitch
attitude_vel = np.array([0.0, 0.0, 0.0])

controller.update_state(com_pos, com_vel, attitude, attitude_vel)
control_output = controller.compute_balance_control()

print(f"Balance control output: {control_output}")
```

### Walking State Machine

A state machine helps coordinate the different phases of walking:

```python title="Walking State Machine"
from enum import Enum

class WalkingState(Enum):
    INITIAL = 1
    SINGLE_SUPPORT_LEFT = 2
    SINGLE_SUPPORT_RIGHT = 3
    DOUBLE_SUPPORT_LEFT = 4
    DOUBLE_SUPPORT_RIGHT = 5
    STANDING = 6
    EMERGENCY_STOP = 7

class WalkingController:
    def __init__(self):
        self.current_state = WalkingState.INITIAL
        self.next_foot_support = "right"  # Start with right foot in single support
        self.step_counter = 0
        self.is_walking = False
        
        # Timing parameters
        self.single_support_duration = 0.8  # seconds
        self.double_support_duration = 0.2  # seconds
        self.current_phase_time = 0.0
        self.total_step_time = self.single_support_duration + self.double_support_duration
    
    def update(self, dt):
        """Update the walking controller state"""
        # Update phase timer
        self.current_phase_time += dt
        
        # Transition between walking states based on time and conditions
        if self.is_walking:
            if self.current_state == WalkingState.INITIAL:
                # Start walking with appropriate foot
                self.current_state = WalkingState.SINGLE_SUPPORT_RIGHT \
                    if self.next_foot_support == "right" else WalkingState.SINGLE_SUPPORT_LEFT
            
            elif self.current_state == WalkingState.SINGLE_SUPPORT_LEFT:
                if self.current_phase_time >= self.single_support_duration:
                    self.current_phase_time = 0.0
                    self.current_state = WalkingState.DOUBLE_SUPPORT_RIGHT
                    self.next_foot_support = "left"  # Next step will be left foot
            
            elif self.current_state == WalkingState.SINGLE_SUPPORT_RIGHT:
                if self.current_phase_time >= self.single_support_duration:
                    self.current_phase_time = 0.0
                    self.current_state = WalkingState.DOUBLE_SUPPORT_LEFT
                    self.next_foot_support = "right"  # Next step will be right foot
            
            elif self.current_state == WalkingState.DOUBLE_SUPPORT_LEFT:
                if self.current_phase_time >= self.double_support_duration:
                    self.current_phase_time = 0.0
                    self.current_state = WalkingState.SINGLE_SUPPORT_LEFT
                    self.step_counter += 1
            
            elif self.current_state == WalkingState.DOUBLE_SUPPORT_RIGHT:
                if self.current_phase_time >= self.double_support_duration:
                    self.current_phase_time = 0.0
                    self.current_state = WalkingState.SINGLE_SUPPORT_RIGHT
                    self.step_counter += 1
        else:
            # If walking is not active, transition to standing
            self.current_state = WalkingState.STANDING
            self.current_phase_time = 0.0
    
    def start_walking(self):
        """Initiate walking motion"""
        self.is_walking = True
        self.step_counter = 0
    
    def stop_walking(self):
        """Stop walking motion"""
        self.is_walking = False
    
    def get_current_support_state(self):
        """Get information about current support state"""
        if self.current_state in [WalkingState.SINGLE_SUPPORT_LEFT, WalkingState.DOUBLE_SUPPORT_LEFT]:
            support_foot = "left"
        elif self.current_state in [WalkingState.SINGLE_SUPPORT_RIGHT, WalkingState.DOUBLE_SUPPORT_RIGHT]:
            support_foot = "right"
        else:
            support_foot = "none"
        
        is_double_support = self.current_state.name.startswith("DOUBLE_SUPPORT")
        
        return {
            "support_foot": support_foot,
            "is_double_support": is_double_support,
            "phase_progress": self.current_phase_time / self.total_step_time
        }

# Example usage
walker = WalkingController()
walker.start_walking()

# Simulate walking
for i in range(100):  # Simulate 100 control steps
    # Assuming 100Hz control rate, dt = 0.01
    dt = 0.01
    walker.update(dt)
    
    state_info = walker.get_current_support_state()
    print(f"Step {walker.step_counter}, State: {walker.current_state.name}, "
          f"Support: {state_info['support_foot']}, "
          f"Progress: {state_info['phase_progress']:.2f}")
    
    # Stop after 5 steps for this demo
    if walker.step_counter >= 5:
        break
```

## ROS 2 Implementation for Walking Control

### Integration with ROS 2 and Control Systems

```python title="ROS 2 Walking Controller Node"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidWalkingController(Node):
    def __init__(self):
        super().__init__('humanoid_walking_controller')
        
        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Create publishers for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )
        
        # Initialize walking controller
        self.walking_controller = WalkingController()
        self.balance_controller = BalanceController()
        
        # Initialize joint positions
        self.joint_positions = {}
        self.joint_velocities = {}
        
        # Initialize IMU data
        self.imu_orientation = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = None
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz control
        
        self.get_logger().info('Humanoid Walking Controller initialized')
    
    def joint_state_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]
    
    def imu_callback(self, msg):
        """Update IMU information"""
        self.imu_orientation = msg.orientation
        self.imu_angular_velocity = msg.angular_velocity
        self.imu_linear_acceleration = msg.linear_acceleration
    
    def control_loop(self):
        """Main control loop for walking and balance"""
        # Update walking state machine
        self.walking_controller.update(0.01)  # dt = 0.01s
        
        # Estimate CoM position and velocity (simplified)
        # In a real implementation, this would use forward kinematics
        com_pos = np.array([0.0, 0.0, 0.8])
        com_vel = np.array([0.0, 0.0, 0.0])
        
        # Extract orientation from IMU
        if self.imu_orientation:
            # Convert quaternion to roll/pitch/yaw (simplified)
            attitude = np.array([0.0, 0.0, 0.0])  # This would be properly calculated
            attitude_vel = np.array([0.0, 0.0, 0.0])  # This would come from IMU angular velocity
        else:
            attitude = np.zeros(3)
            attitude_vel = np.zeros(3)
        
        # Update balance controller
        self.balance_controller.update_state(com_pos, com_vel, attitude, attitude_vel)
        balance_control_output = self.balance_controller.compute_balance_control()
        
        # Generate joint commands based on walking and balance control
        joint_commands = self.generate_joint_commands(
            self.walking_controller.get_current_support_state(),
            balance_control_output
        )
        
        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.joint_cmd_pub.publish(cmd_msg)
    
    def generate_joint_commands(self, support_state, balance_control):
        """Generate joint commands based on support state and balance"""
        # This is a simplified implementation
        # In reality, this would be much more complex and involve:
        # 1. Inverse kinematics to achieve desired CoM position
        # 2. Joint space inverse dynamics
        # 3. Proper gait control
        
        # Placeholder: return neutral positions with small adjustments for balance
        neutral_positions = [0.0] * 10  # 10 joints for this example
        
        # Apply balance corrections
        # These would be calculated based on inverse kinematics
        for i in range(min(len(neutral_positions), len(balance_control))):
            neutral_positions[i] += balance_control[i] * 0.01  # Small correction factor
        
        return neutral_positions

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidWalkingController()
    
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

## Simulation and Testing

### Gazebo Simulation Setup

To properly test walking controllers, simulation is essential. A typical setup includes:

1. **Humanoid Robot Model**: Accurate URDF with realistic mass and inertia
2. **Physics Engine**: Properly configured for contact simulation
3. **Sensors**: IMUs, joint encoders, force/torque sensors
4. **Ground Plane**: With appropriate friction coefficients

## Hands-On Exercise

1. Implement the ZMP-based walking pattern generator with a real humanoid URDF model
2. Simulate the walking pattern in Gazebo
3. Add balance control to handle small disturbances
4. Test the walking controller on different terrains
5. Adjust controller parameters to improve stability

## Summary

Bipedal walking and balance control is one of the most sophisticated challenges in humanoid robotics. The ZMP theory provides a solid foundation for creating stable walking patterns, while feedback control systems help the robot respond to disturbances in real-time. Successful implementation requires careful integration of pattern generation, balance control, and robot dynamics. The combination of preview control for trajectory generation and feedback control for disturbance rejection forms the core of modern humanoid walking control systems.