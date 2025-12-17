---
title: "Capstone Implementation Guide"
sidebar_label: "Implementation Guide"
description: "Step-by-step guide for implementing the capstone project"
keywords: [capstone, implementation, guide, humanoid, robotics, ai]
---

# Capstone Implementation Guide

## Getting Started

This guide provides detailed steps to implement the autonomous humanoid robot system. The approach is iterative, building and integrating components gradually rather than attempting to implement everything at once.

## Development Environment Setup

### Prerequisites

1. **Ubuntu 22.04 LTS** (required for ROS 2 Humble)
2. **ROS 2 Humble Hawksbill** with all desktop packages
3. **Gazebo Garden** for simulation
4. **Python 3.8+** with required packages
5. **NVIDIA GPU** with CUDA support (for Isaac ROS packages)

### Required Software Installation

```bash
# Install ROS 2 Humble
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install Gazebo
curl -sSL http://get.gazebosim.org | sh
sudo apt-get install gz-harmonic

# Install Python packages for VLA components
pip3 install openai-whisper torch torchvision torchaudio openai
pip3 install rclpy numpy opencv-python
```

## Project Structure

Create the following workspace structure:

```bash
# Create workspace
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws

# Create the main robot package
cd src
ros2 pkg create --build-type ament_python robot_command_interface
ros2 pkg create --build-type ament_python perception_system
ros2 pkg create --build-type ament_python navigation_system
ros2 pkg create --build-type ament_python manipulation_system
ros2 pkg create --build-type ament_python vla_integration

cd ~/capstone_ws
colcon build --packages-select robot_command_interface perception_system navigation_system manipulation_system vla_integration
source install/setup.bash
```

## Step 1: Create the Robot Model

First, let's create a humanoid robot model with URDF:

```xml title="~/capstone_ws/src/robot_description/urdf/humanoid_robot.urdf.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">
  <!-- Base Link -->
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
      <material name="gray"/>
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
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1"/>
      </material>
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
      <material name="red">
        <color rgba="0.8 0.0 0.0 1"/>
      </material>
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

  <!-- Right Elbow -->
  <link name="right_elbow">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_elbow"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="20" velocity="2"/>
  </joint>

  <!-- Right Wrist -->
  <link name="right_wrist">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.08"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_elbow"/>
    <child link="right_wrist"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="2"/>
  </joint>

  <!-- Right Hand (simplified as a single link with gripper) -->
  <link name="right_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="flesh">
        <color rgba="0.8 0.5 0.3 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_hand_joint" type="fixed">
    <parent link="right_wrist"/>
    <child link="right_hand"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Left Arm (symmetric to right) -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.15"/>
      </geometry>
      <material name="red"/>
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

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.12 0 0.1" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>

  <link name="left_elbow">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_elbow"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="20" velocity="2"/>
  </joint>

  <link name="left_wrist">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.08"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_elbow"/>
    <child link="left_wrist"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="2"/>
  </joint>

  <link name="left_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="flesh"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hand_joint" type="fixed">
    <parent link="left_wrist"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_hip">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.15"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 0.8 0.0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="-0.07 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="2"/>
  </joint>

  <link name="right_knee">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.35"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_knee"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.5" effort="30" velocity="2"/>
  </joint>

  <link name="right_ankle">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_knee"/>
    <child link="right_ankle"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_foot_joint" type="fixed">
    <parent link="right_ankle"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Left Leg (symmetric) -->
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.15"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.07 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="2"/>
  </joint>

  <link name="left_knee">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.35"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_knee"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.5" effort="30" velocity="2"/>
  </joint>

  <link name="left_ankle">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_knee"/>
    <child link="left_ankle"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_foot_joint" type="fixed">
    <parent link="left_ankle"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
  </joint>

  <!-- Sensors -->
  <gazebo reference="head">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.089</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- ROS Control Interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="right_shoulder_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_elbow_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wrist_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <!-- Add interfaces for other joints -->
  </ros2_control>
</robot>
```

## Step 2: Create the Simulation Environment

Create a simple world file for Gazebo:

```xml title="~/capstone_ws/src/worlds/simple_world.sdf"
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add objects for manipulation -->
    <model name="table1">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="cup">
      <pose>2.2 0.1 0.85 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.04</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.04</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.2</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="book">
      <pose>2.0 -0.2 0.85 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.15 0.2 0.03</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.15 0.2 0.03</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.3</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add walls to create rooms -->
    <model name="wall1">
      <pose>-3 0 1.5 0 0 0</pose>
      <link name="wall1_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 6 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 6 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall2">
      <pose>0 -3 1.5 0 0 1.5707</pose>
      <link name="wall2_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 6 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 6 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Step 3: Implement the Voice Command Interface

Create a ROS 2 node for voice command processing:

```python title="~/capstone_ws/src/robot_command_interface/robot_command_interface/voice_command_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import whisper
import pyaudio
import numpy as np
import queue
import threading
import time
import subprocess

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Publisher for recognized commands
        self.command_pub = self.create_publisher(String, 'robot_commands', 10)
        
        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'voice_status', 10)
        
        # Service to start/stop listening
        self.start_service = self.create_service(
            Trigger, 'start_voice_recognition', self.start_recognition_callback
        )
        
        self.stop_service = self.create_service(
            Trigger, 'stop_voice_recognition', self.stop_recognition_callback
        )
        
        # Initialize Whisper model
        self.get_logger().info('Loading Whisper model...')
        
        # Use a smaller model for faster processing
        self.model_size = "base"  # Use "tiny" for even faster processing
        try:
            self.model = whisper.load_model(self.model_size)
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.model = None
        
        # Audio parameters
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.audio_queue = queue.Queue()
        
        # Recognition state
        self.recording = False
        self.recognition_thread = None
        
        # Audio stream (initialize when needed)
        self.audio_stream = None
        self.pyaudio_instance = None
        
        self.get_logger().info('Voice Command Node initialized')
    
    def start_recognition_callback(self, request, response):
        """Start voice recognition service callback"""
        if not self.model:
            response.success = False
            response.message = 'Whisper model not loaded'
            return response
        
        if self.recording:
            response.success = False
            response.message = 'Already recording'
            return response
        
        self.start_voice_recognition()
        response.success = True
        response.message = 'Voice recognition started'
        
        status_msg = String()
        status_msg.data = 'Voice recognition started'
        self.status_pub.publish(status_msg)
        
        return response
    
    def stop_recognition_callback(self, request, response):
        """Stop voice recognition service callback"""
        if not self.recording:
            response.success = False
            response.message = 'Not currently recording'
            return response
        
        self.stop_voice_recognition()
        response.success = True
        response.message = 'Voice recognition stopped'
        
        status_msg = String()
        status_msg.data = 'Voice recognition stopped'
        self.status_pub.publish(status_msg)
        
        return response
    
    def start_voice_recognition(self):
        """Start the voice recognition process"""
        self.recording = True
        self.init_audio()
        
        # Start recognition thread
        self.recognition_thread = threading.Thread(target=self.recognition_loop)
        self.recognition_thread.start()
    
    def stop_voice_recognition(self):
        """Stop the voice recognition process"""
        self.recording = False
        
        if self.recognition_thread:
            self.recognition_thread.join()
        
        self.cleanup_audio()
    
    def init_audio(self):
        """Initialize audio input"""
        self.pyaudio_instance = pyaudio.PyAudio()
        self.audio_stream = self.pyaudio_instance.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
    
    def cleanup_audio(self):
        """Clean up audio resources"""
        if self.audio_stream:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        
        if self.pyaudio_instance:
            self.pyaudio_instance.terminate()
    
    def recognition_loop(self):
        """Main recognition loop"""
        audio_buffer = np.array([])
        silence_threshold = 500  # Adjust based on your environment
        speech_threshold_duration = 2.0  # seconds of speech needed before processing
        silence_duration = 1.0  # seconds of silence before processing
        
        last_voice_time = time.time()
        started_recording = False
        
        while self.recording:
            # Read audio data
            try:
                data = self.audio_stream.read(self.chunk_size, exception_on_overflow=False)
                audio_chunk = np.frombuffer(data, dtype=np.int16)
            except:
                continue
            
            # Check if audio chunk contains voice (simple energy threshold)
            energy = np.sqrt(np.mean(audio_chunk**2))
            has_voice = energy > silence_threshold
            
            if has_voice:
                # Add chunk to buffer
                audio_buffer = np.concatenate([audio_buffer, audio_chunk])
                last_voice_time = time.time()
                
                if not started_recording:
                    started_recording = True
                    self.get_logger().info('Started recording speech')
            else:
                # No voice detected
                if started_recording:
                    # Check if we've had sufficient silence to process
                    if time.time() - last_voice_time > silence_duration and len(audio_buffer) > 0:
                        # Process the buffered audio
                        self.process_audio_buffer(audio_buffer)
                        audio_buffer = np.array([])
                        started_recording = False
                        self.get_logger().info('Processing audio buffer')
        
        # Process remaining audio if any
        if len(audio_buffer) > 0:
            self.process_audio_buffer(audio_buffer)
    
    def process_audio_buffer(self, audio_buffer):
        """Process the accumulated audio buffer"""
        if len(audio_buffer) < self.sample_rate * 0.5:  # Less than 0.5 seconds
            return  # Too short to be a meaningful command
        
        try:
            # Normalize audio
            audio_normalized = audio_buffer.astype(np.float32) / 32768.0
            
            # Transcribe using Whisper
            result = self.model.transcribe(audio_normalized)
            recognized_text = result.get("text", "").strip()
            
            if recognized_text:
                self.get_logger().info(f'Recognized: "{recognized_text}"')
                
                # Publish the recognized command
                cmd_msg = String()
                cmd_msg.data = recognized_text
                self.command_pub.publish(cmd_msg)
                
                # Log to console as well
                self.get_logger().info(f'Published command: {recognized_text}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    voice_node = VoiceCommandNode()
    
    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        voice_node.get_logger().info('Interrupted, shutting down')
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 4: Implement LLM-Based Task Planning

Create a node for processing commands with LLMs:

```python title="~/capstone_ws/src/vla_integration/vla_integration/llm_planning_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
import json
import openai
import time

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')
        
        # Set your OpenAI API key (in practice, use secure methods to store this)
        # openai.api_key = "YOUR_API_KEY_HERE"  # Don't hardcode in production
        openai.api_key = self.get_parameter_or_set_default('openai_api_key', '')
        
        # Publisher for generated plans
        self.plan_pub = self.create_publisher(String, 'robot_plans', 10)
        
        # Subscription to voice commands
        self.command_sub = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )
        
        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'llm_status', 10)
        
        # Robot capabilities for LLM to reference
        self.robot_capabilities = [
            "navigate to location",
            "navigate to object",
            "grasp object",
            "release object",
            "move arm to position",
            "open gripper", 
            "close gripper",
            "wait",
            "stop"
        ]
        
        # Known objects in environment
        self.known_objects = [
            "cup", "book", "box", "bottle", "chair", "table", "shelf", "refrigerator"
        ]
        
        self.get_logger().info('LLM Planning Node initialized')
    
    def command_callback(self, msg):
        """Process incoming voice command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Generate a plan using LLM
        plan = self.generate_plan_with_llm(command)
        
        if plan:
            # Publish the generated plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            
            self.get_logger().info('Published plan to robot_plans topic')
        else:
            self.get_logger().error('Failed to generate plan')
    
    def generate_plan_with_llm(self, command):
        """Generate a task plan using LLM"""
        prompt = f"""
        You are a robotic task planner. The robot has the following capabilities: {', '.join(self.robot_capabilities)}.
        The environment contains these objects: {', '.join(self.known_objects)}.
        
        User command: "{command}"
        
        Generate a step-by-step plan that the robot can execute to fulfill this command.
        Each step should include an action from the capabilities list.
        
        Respond in JSON format:
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
            ],
            "confidence": 0.8
        }}
        
        Make the plan specific and executable. If the command is ambiguous, ask for clarification.
        """
        
        try:
            # Call the OpenAI API
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
            )
            
            # Extract the response content
            content = response.choices[0].message['content']
            
            # Parse the JSON response
            plan = json.loads(content)
            
            self.get_logger().info(f'Generated plan: {plan["thoughts"]}')
            return plan
            
        except Exception as e:
            self.get_logger().error(f'Error with LLM: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    planning_node = LLMPlanningNode()
    
    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        planning_node.get_logger().info('Interrupted, shutting down')
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: Implement the Central Control Node

Create a central node that coordinates all components:

```python title="~/capstone_ws/src/robot_command_interface/robot_command_interface/central_control_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import json
import time

class CentralControlNode(Node):
    def __init__(self):
        super().__init__('central_control_node')
        
        # Subscribe to the LLM-generated plans
        self.plan_sub = self.create_subscription(
            String,
            'robot_plans',
            self.plan_callback,
            10
        )
        
        # Subscribe to voice commands (for fallback)
        self.voice_sub = self.create_subscription(
            String,
            'robot_commands',
            self.voice_command_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        
        # Service clients (for calling other nodes)
        from rclpy.qos import QoSProfile
        qos_profile = QoSProfile(depth=10)
        self.voice_start_client = self.create_client(String, 'start_voice_recognition')
        self.voice_stop_client = self.create_client(String, 'stop_voice_recognition')
        
        # Robot state
        self.current_plan = None
        self.plan_execution_active = False
        self.current_step = 0
        
        # Robot physical parameters
        self.wheel_radius = 0.05  # meters
        self.axle_length = 0.3   # meters (distance between wheels)
        
        self.get_logger().info('Central Control Node initialized')
    
    def plan_callback(self, msg):
        """Process incoming plan from LLM"""
        try:
            plan_data = json.loads(msg.data)
            self.current_plan = plan_data
            self.current_step = 0
            self.plan_execution_active = True
            
            self.get_logger().info(f'Starting execution of plan with {len(plan_data.get("plan", []))} steps')
            
            # Start executing the plan
            self.execute_plan()
            
        except json.JSONDecodeError:
            self.get_logger().error('Received invalid JSON plan')
        except Exception as e:
            self.get_logger().error(f'Error processing plan: {e}')
    
    def voice_command_callback(self, msg):
        """Handle voice commands directly if no plan is active"""
        command = msg.data.lower()
        
        if not self.plan_execution_active:
            # For simple commands not requiring full planning
            if 'stop' in command:
                self.stop_robot()
            elif 'forward' in command or 'straight' in command:
                self.move_forward()
            elif 'backward' in command:
                self.move_backward()
            elif 'left' in command or 'turn left' in command:
                self.turn_left()
            elif 'right' in command or 'turn right' in command:
                self.turn_right()
    
    def execute_plan(self):
        """Execute the current plan step by step"""
        if not self.current_plan or not self.plan_execution_active:
            return
        
        plan_steps = self.current_plan.get('plan', [])
        
        for i, step in enumerate(plan_steps):
            self.get_logger().info(f'Executing step {i+1}: {step["description"]}')
            
            success = self.execute_single_step(step)
            
            if not success:
                self.get_logger().error(f'Step {i+1} failed, stopping plan execution')
                break
            
            # Small delay between steps
            time.sleep(0.5)
        
        self.plan_execution_active = False
        self.current_plan = None
        
        status_msg = String()
        status_msg.data = 'Plan execution completed'
        self.status_pub.publish(status_msg)
    
    def execute_single_step(self, step):
        """Execute a single step of the plan"""
        action = step.get('action', '')
        
        try:
            if action == 'navigate to location':
                params = step.get('parameters', {})
                x = params.get('x', 0.0)
                y = params.get('y', 0.0)
                self.navigate_to(x, y)
                
            elif action == 'navigate to object':
                params = step.get('parameters', {})
                object_name = params.get('object', '')
                self.navigate_to_object(object_name)
                
            elif action == 'grasp object':
                params = step.get('parameters', {})
                object_name = params.get('object', '')
                self.grasp_object(object_name)
                
            elif action == 'release object':
                self.release_object()
                
            elif action == 'move arm to position':
                # This would involve more complex arm control
                pass
                
            elif action == 'stop':
                self.stop_robot()
                
            elif action == 'wait':
                duration = step.get('parameters', {}).get('duration', 1.0)
                time.sleep(duration)
                
            else:
                self.get_logger().warning(f'Unknown action: {action}')
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error executing step: {e}')
            return False
    
    def navigate_to(self, x, y):
        """Simple navigation to a specific coordinate"""
        self.get_logger().info(f'Navigating to ({x}, {y})')
        
        # Create a simple movement command (in practice, this would interface with Nav2)
        twist = Twist()
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        twist.angular.z = 0.0  # No turning
        
        # Publish for a set duration (in practice, use proper navigation with feedback)
        duration = 5  # seconds
        start_time = time.time()
        
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        self.stop_robot()
    
    def navigate_to_object(self, object_name):
        """Navigate to a named object (in simulation, use fixed positions)"""
        self.get_logger().info(f'Navigating to {object_name}')
        
        # In a real implementation, this would use perception to locate the object
        # For simulation, we'll use predefined positions
        object_positions = {
            'cup': (2.2, 0.1, 0.85),
            'book': (2.0, -0.2, 0.85),
            'table': (2.0, 0.0, 0.0),
        }
        
        if object_name in object_positions:
            pos = object_positions[object_name]
            self.navigate_to(pos[0], pos[1])
        else:
            self.get_logger().warning(f'Unknown object: {object_name}')
    
    def grasp_object(self, object_name):
        """Simulate grasping an object"""
        self.get_logger().info(f'Attempting to grasp {object_name}')
        
        # In a real implementation, this would control the robot's gripper
        # Publish joint commands to close the gripper
        joint_state = JointState()
        joint_state.name = ['right_hand_joint', 'left_hand_joint']  # Simplified
        joint_state.position = [0.01, 0.01]  # Close gripper
        
        self.joint_cmd_pub.publish(joint_state)
    
    def release_object(self):
        """Simulate releasing an object"""
        self.get_logger().info('Releasing object')
        
        # Publish joint commands to open the gripper
        joint_state = JointState()
        joint_state.name = ['right_hand_joint', 'left_hand_joint']
        joint_state.position = [0.05, 0.05]  # Open gripper
        
        self.joint_cmd_pub.publish(joint_state)
    
    def stop_robot(self):
        """Stop all robot motion"""
        self.get_logger().info('Stopping robot')
        
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.2  # 0.2 m/s forward
        self.cmd_vel_pub.publish(twist)
        
        # Move for 3 seconds then stop
        time.sleep(3)
        self.stop_robot()
    
    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -0.2  # 0.2 m/s backward
        self.cmd_vel_pub.publish(twist)
        
        # Move for 3 seconds then stop
        time.sleep(3)
        self.stop_robot()
    
    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.angular.z = 0.5  # 0.5 rad/s counter-clockwise
        self.cmd_vel_pub.publish(twist)
        
        # Turn for 1 second then stop
        time.sleep(1)
        self.stop_robot()
    
    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.angular.z = -0.5  # 0.5 rad/s clockwise
        self.cmd_vel_pub.publish(twist)
        
        # Turn for 1 second then stop
        time.sleep(1)
        self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    
    control_node = CentralControlNode()
    
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info('Interrupted, shutting down')
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 6: Launch Configuration

Create launch files to bring up all components together:

```python title="~/capstone_ws/src/robot_command_interface/launch/capstone_demo.launch.py"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Voice Command Node
        Node(
            package='robot_command_interface',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen',
        ),
        
        # LLM Planning Node
        Node(
            package='vla_integration',
            executable='llm_planning_node',
            name='llm_planning_node',
            parameters=[
                {'openai_api_key': 'YOUR_API_KEY_HERE'}  # Use secure methods in production
            ],
            output='screen',
        ),
        
        # Central Control Node
        Node(
            package='robot_command_interface',
            executable='central_control_node',
            name='central_control_node',
            output='screen',
        ),
        
        # Perception System Node (placeholder for now)
        Node(
            package='perception_system',
            executable='object_detection_node', 
            name='object_detection_node',
            output='screen',
        ),
        
        # Navigation System Node (placeholder for now)
        Node(
            package='navigation_system',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
        ),
    ])
```

## Building and Running the System

1. Build the workspace:
```bash
cd ~/capstone_ws
colcon build --packages-select robot_command_interface perception_system navigation_system manipulation_system vla_integration
source install/setup.bash
```

2. Launch the system:
```bash
ros2 launch robot_command_interface capstone_demo.launch.py
```

3. Start voice recognition separately:
```bash
ros2 service call /start_voice_recognition std_srvs/srv/Trigger
```

This implementation guide provides a foundational system that integrates the key components of the capstone project. Each component can be enhanced with more sophisticated algorithms and proper error handling as needed for a full implementation.