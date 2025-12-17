---
title: "Week 3 - rclpy Integration and URDF for Humanoids"
sidebar_label: "Week 3: rclpy and URDF"
description: "Practical Python integration with rclpy and creating robot models with URDF for humanoid robotics"
keywords: [rclpy, urdf, python, robot description, ros2, robotics, humanoid]
---

# Week 3: rclpy Integration and URDF for Humanoids

## Learning Outcomes

By the end of this week, you will be able to:
- Develop complex ROS 2 nodes using Python and rclpy
- Create and parse URDF files for robot description
- Implement parameter handling in ROS 2 nodes
- Design robot models suitable for humanoid applications
- Use TF (Transforms) for coordinate frame management

## Advanced rclpy Concepts

### Parameter Handling

Parameters allow runtime configuration of nodes without recompilation. They're especially useful for humanoid robots that may need to adjust behavior based on calibration or environmental conditions.

```python title="Parameter Handling in ROS 2 Nodes"
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with defaults
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('base_width', 0.5)
        self.declare_parameter('max_velocity', 1.0)
        
        # Access parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.PARAMETER_DOUBLE:
                self.max_velocity = param.value
                self.get_logger().info(f'Max velocity updated to {param.value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Timer-Based Execution

Timers enable periodic execution of functions, crucial for control loops in humanoid robots:

```python title="Timer-Based Control in ROS 2"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Create publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        # Create subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Create timer for control loop (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.current_joint_states = JointState()
        
    def joint_state_callback(self, msg):
        self.current_joint_states = msg
        
    def control_loop(self):
        # Implement your control algorithm here
        cmd_msg = Float64MultiArray()
        
        # Example: simple proportional controller
        target_positions = [0.0, 0.5, -0.5]  # desired joint positions
        
        for i, current_pos in enumerate(self.current_joint_states.position):
            if i < len(target_positions):
                error = target_positions[i] - current_pos
                cmd_msg.data.append(error * 1.0)  # simple P-controller
            else:
                cmd_msg.data.append(0.0)
        
        self.joint_cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Threading and Async Execution

For complex humanoid applications, you may need to run multiple operations concurrently:

```python title="Threading in ROS 2 Nodes"
import rclpy
from rclpy.node import Node
import threading
import time
from std_msgs.msg import String

class ThreadingNode(Node):
    def __init__(self):
        super().__init__('threading_node')
        
        # Shared data between threads
        self.lock = threading.Lock()
        self.shared_data = {'counter': 0, 'message': 'default'}
        
        # Start background thread
        self.background_thread = threading.Thread(target=self.background_work)
        self.background_thread.start()
        
        # Timer for publishing data from shared resource
        self.publish_timer = self.create_timer(0.5, self.publish_data)
        
        # Publisher
        self.pub = self.create_publisher(String, 'threading_topic', 10)
        
    def background_work(self):
        counter = 0
        while rclpy.ok():
            with self.lock:
                self.shared_data['counter'] = counter
                self.shared_data['message'] = f'Updated at {time.time()}'
            counter += 1
            time.sleep(1)
    
    def publish_data(self):
        with self.lock:
            msg = String()
            msg.data = f"Counter: {self.shared_data['counter']}, Message: {self.shared_data['message']}"
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThreadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Understanding URDF (Unified Robot Description Format)

URDF is XML-based format for representing robot models, including kinematic structure, visual appearance, and physical properties. It's essential for humanoid robots to define their complex structure.

### URDF Structure

A basic URDF file contains:
- Robot name and basic properties
- Links: rigid bodies with visual, collision, and inertial properties
- Joints: connections between links that define motion constraints
- Transmissions: mapping between ROS controls and joints (for hardware interfaces)

### Basic URDF Example

```xml title="Basic URDF Robot Model"
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.6"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots require complex URDF models with many degrees of freedom and careful mass distribution:

```xml title="Example Humanoid Robot URDF"
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base link - pelvis -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.15"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.275" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="2"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.12 0 0.1" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <!-- Additional links and joints would continue for full humanoid -->
  <!-- (left arm, legs, hands, feet, etc.) -->

  <!-- ROS Control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <!-- Joint interfaces would be defined here -->
  </ros2_control>
</robot>
```

## Working with URDF in Python

ROS 2 provides ways to load, parse, and work with URDF models in Python:

```python title="Loading and Processing URDF in Python"
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
import os

class URDFProcessor(Node):
    def __init__(self):
        super().__init__('urdf_processor')
        
        # Load URDF from file or parameter
        urdf_path = os.path.join(os.path.expanduser('~'), 'robot.urdf')
        
        try:
            with open(urdf_path, 'r') as f:
                urdf_string = f.read()
            
            # Parse the URDF
            robot = URDF.from_xml_string(urdf_string)
            
            self.get_logger().info(f'Loaded robot: {robot.name}')
            self.get_logger().info(f'Number of links: {len(robot.links)}')
            self.get_logger().info(f'Number of joints: {len(robot.joints)}')
            
            # Process joint information
            for joint in robot.joints:
                self.get_logger().info(f'Joint: {joint.name}, Type: {joint.type}')
                
            # Process link information
            for link in robot.links:
                self.get_logger().info(f'Link: {link.name}')
                if link.inertial:
                    mass = link.inertial.mass
                    self.get_logger().info(f'  Mass: {mass}')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing URDF: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = URDFProcessor()
    rclpy.spin_once(node, timeout_sec=0)  # Process once and exit
    node.destroy_node()
    rclpy.shutdown()
```

## Transform Management (TF)

TF (Transforms) is ROS's system for tracking coordinate frames and their relationships over time. For humanoid robots with many joints, TF is essential:

```python title="Using TF in ROS 2 Python Nodes"
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to broadcast transforms at regular intervals
        self.timer = self.create_timer(0.1, self.broadcast_transforms)
        
    def broadcast_transforms(self):
        # Create a transform from base_link to sensor_frame
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'sensor_frame'
        
        # Set the translation
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        
        # Set the rotation (as quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### TF with URDF

When using URDF models, joint states automatically update the TF tree:

```python title="Joint State Publisher with URDF"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz
        
        # Initialize joint values
        self.joint_positions = [0.0] * 10  # 10 joints in our example
        self.start_time = time.time()
        
    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                   'joint6', 'joint7', 'joint8', 'joint9', 'joint10']
        
        # Update positions with a simple oscillating pattern
        current_time = time.time() - self.start_time
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * math.sin(current_time + i * 0.5)
        
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Publish the message
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Best Practices for Humanoid URDF Models

### Kinematic Chain Design
- Model each limb as a separate kinematic chain
- Ensure closed loops are handled properly (e.g., when both feet are on the ground)
- Use appropriate joint types (revolute for rotating, prismatic for sliding, fixed for non-moving)

### Mass Distribution
- Accurately model mass and inertia properties
- Use realistic values based on physical measurements
- Consider the impact of mass distribution on balance and control

### Visual and Collision Models
- Use simplified collision models for performance
- Detailed visual models for rendering
- Consider multiple levels of detail for different applications

### Joint Limits and Safety
- Set appropriate joint limits to prevent damage
- Include safety margins in joint ranges
- Consider software and hardware limits

## Hands-On Exercise

Create a simplified URDF model for a humanoid robot with at least 10 joints (torso, head, 2 arms, 2 legs). Implement a Python node that publishes joint states for this robot, and visualize the robot in RViz2. Add a control node that moves the joints to follow a simple trajectory.

## Summary

Combining Python with rclpy and URDF provides a powerful framework for humanoid robot development. Parameter handling, timer-based execution, and threading allow sophisticated control implementations, while URDF models enable accurate representation of complex humanoid kinematics. Understanding these tools is essential for building effective humanoid robot applications with realistic physical models and responsive control systems.