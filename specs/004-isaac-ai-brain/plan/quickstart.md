# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This guide provides setup instructions for the digital twin system using NVIDIA Isaac Sim, Isaac ROS, and Unity integration. It covers the fundamental concepts and initial setup required to run the examples in this module.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA GPU with latest drivers
- Isaac Sim 2023.1+
- Unity 2021.3 LTS or newer
- Python 3.8+

## Installation Steps

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
```

### 2. Install Isaac Sim
```bash
# Download and install Isaac Sim from NVIDIA Developer website
# Follow the installation guide for your platform
# Verify installation:
cd ~/isaac_sim && ./runheadless.py -- --summary
```

### 3. Install Isaac ROS Packages
```bash
# Create workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws
source /opt/ros/humble/setup.bash

# Install Isaac ROS packages
sudo apt install -y ros-humble-isaac-ros-* ros-humble-ros-gz
```

### 4. Set up Unity with ROS-TCP-Endpoint
1. Install Unity Hub and Unity 2021.3 LTS or newer
2. Clone the ROS-TCP-Endpoint repository:
```bash
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```
3. Import the ROS-TCP-Endpoint Unity package into your Unity project

## Running the Examples

### Basic Isaac Sim Physics Example
```bash
# Terminal 1: Start Isaac Sim
cd ~/isaac_sim
./runheadless.py -- -v -x default # Use appropriate scene

# Terminal 2: Verify the physics simulation
source ~/isaac_ws/install/setup.bash
# Run a test node that communicates with Isaac Sim
```

### Isaac ROS Sensor Simulation Example
```bash
# Run Isaac ROS visual slam example
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### Unity Bridge Example
```bash
# 1. Start Isaac Sim with a humanoid simulation
# 2. Start ROS-TCP-Endpoint bridge
cd ~/ROS-TCP-Endpoint
python3 ros_tcp_endpoint/bridge.py

# 3. Start Unity scene configured to connect to the TCP endpoint
# 4. Verify robot state synchronization between Isaac Sim and Unity
```

## Hardware Requirements Warning

:::caution
This module requires specific hardware to function properly:

- **NVIDIA RTX GPU**: For Isaac Sim photorealistic rendering and Isaac ROS hardware acceleration
- **Jetson Orin**: For sim-to-real transfer and deployment validation on edge platforms

Without these NVIDIA hardware components, students will not be able to execute the hands-on examples and benchmarks described in this module.
:::

## Troubleshooting

### Common Issues

1. **Isaac Sim won't start**
   - Verify NVIDIA GPU drivers are up to date
   - Check that CUDA toolkit is properly installed
   - Run Isaac Sim with verbose logging: `./runheadless.py -- -v`

2. **Isaac ROS packages missing**
   - Check that you installed all Isaac ROS packages: `apt list --installed | grep isaac-ros`
   - Verify ROS 2 Humble is properly sourced

3. **Unity connection fails**
   - Check that ROS-TCP-Endpoint is running and listening on the correct port
   - Verify firewall settings allow communication on the configured port
   - Confirm IP address configuration matches between Unity and ROS bridge

## Performance Verification

To verify your setup meets the performance requirements:
1. Isaac Sim should run physics simulation at >50Hz
2. Unity visualization should maintain <100ms latency for state updates
3. Control loops should maintain <10ms latency

Use the following command to check simulation performance:
```bash
# In Isaac Sim console or through ROS topics
ros2 topic hz /clock  # Should show 50-1000Hz depending on simulation settings
```