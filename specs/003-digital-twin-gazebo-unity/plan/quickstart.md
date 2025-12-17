# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: Module 2: The Digital Twin (Gazebo & Unity)
**Branch**: 003-digital-twin-gazebo-unity
**Created**: 2025-12-15

## Prerequisites

Before starting with this module, ensure you have:

- Ubuntu 22.04 LTS or Windows 10/11 with WSL2
- ROS 2 Humble Hawksbill installed with Python development tools
- Gazebo Harmonic installed
- Unity Hub with Unity 2021.3 LTS or newer
- Python 3.8 or higher
- Docusaurus development environment (Node.js 16+)
- Git version control system

## Environment Setup

### 1. Install ROS 2 Humble Hawksbill

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
sudo apt install -y ros-humble-tf2-tools ros-humble-rviz2

# Install additional sensor packages
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces

# Initialize rosdep
sudo rosdep init
rosdep update

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 2. Set up your workspace

```bash
# Create a workspace directory
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Source ROS 2 environment each time you open a new terminal
source /opt/ros/humble/setup.bash

# Create a colcon workspace
colcon build
source install/setup.bash
```

### 3. Install Python dependencies

```bash
pip3 install lxml pyquaternion transforms3d numpy matplotlib
pip3 install opencv-python sensor-msgs rosidl-parser
```

### 4. Install Unity with ROS-TCP-Endpoint

Unity setup requires manual installation:
1. Download and install Unity Hub from unity.com
2. Install Unity 2021.3 LTS or newer
3. Install the ROS-TCP-Endpoint Unity package through the Package Manager
4. Import standard Assets for robotics simulation

### 5. Set up Docusaurus documentation environment

```bash
npx create-docusaurus@latest docs-site classic
cd docs-site
npm install
```

## Hello World: Basic Digital Twin Setup

Let's create a simple digital twin system with a basic robot model:

### 1. Create SDF Model (simplified humanoid)

```xml
<!-- basic_humanoid.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <pose>0 0 1 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.4</iyy>
          <iyz>0.0</iyz>
          <izz>0.4</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <capsule>
            <radius>0.15</radius>
            <length>0.8</length>
          </capsule>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <capsule>
            <radius>0.15</radius>
            <length>0.8</length>
          </capsule>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### 2. Launch Gazebo with the model

```bash
# Terminal 1: Start Gazebo with the world
source /opt/ros/humble/setup.bash
cd ~/digital_twin_ws
source install/setup.bash
gz sim -r empty.sdf
```

### 3. Create a simple Python script to connect Unity via ROS-TCP

```python
# unity_bridge_demo.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading


class UnityBridgeDemo(Node):
    def __init__(self):
        super().__init__('unity_bridge_demo')
        
        # ROS publisher for sending robot state
        self.state_publisher = self.create_publisher(String, 'unity_robot_state', 10)
        
        # TCP server for Unity communication
        self.tcp_host = '127.0.0.1'
        self.tcp_port = 10000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.socket.bind((self.tcp_host, self.tcp_port))
            self.socket.listen(1)
            self.get_logger().info(f'Unity Bridge listening on {self.tcp_host}:{self.tcp_port}')
            
            # Start TCP listening thread
            self.tcp_thread = threading.Thread(target=self.handle_tcp_connections)
            self.tcp_thread.daemon = True
            self.tcp_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to start TCP server: {str(e)}')
        
        # Timer to publish state updates
        self.timer = self.create_timer(0.1, self.publish_state_update)  # 10Hz
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example joint positions

    def handle_tcp_connections(self):
        """Handle incoming connections from Unity"""
        while True:
            try:
                conn, addr = self.socket.accept()
                self.get_logger().info(f'Unity connected from {addr}')
                
                while True:
                    data = conn.recv(1024).decode('utf-8')
                    if not data:
                        break
                    
                    # Process command from Unity
                    command = json.loads(data)
                    self.process_unity_command(command)
                    
                    # Send current state back to Unity
                    state_msg = json.dumps({
                        'joint_positions': self.joint_positions,
                        'timestamp': self.get_clock().now().seconds_nanoseconds()
                    })
                    conn.sendall(state_msg.encode('utf-8'))
                    
            except Exception as e:
                self.get_logger().error(f'TCP connection error: {str(e)}')
                if conn:
                    conn.close()

    def process_unity_command(self, command):
        """Process commands received from Unity"""
        cmd_type = command.get('type')
        if cmd_type == 'move_joint':
            joint_idx = command.get('joint_index')
            position = command.get('position')
            if joint_idx is not None and 0 <= joint_idx < len(self.joint_positions):
                self.joint_positions[joint_idx] = position
                self.get_logger().info(f'Moved joint {joint_idx} to {position}')

    def publish_state_update(self):
        """Publish robot state for other ROS nodes"""
        state_msg = String()
        state_msg.data = json.dumps({
            'joint_positions': self.joint_positions,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        })
        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    unity_bridge = UnityBridgeDemo()
    
    try:
        rclpy.spin(unity_bridge)
    except KeyboardInterrupt:
        unity_bridge.get_logger().info('Shutting down Unity bridge')
    finally:
        unity_bridge.socket.close()
        unity_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Run the demo

Terminal 1 (Gazebo):
```bash
source /opt/ros/humble/setup.bash
cd ~/digital_twin_ws
source install/setup.bash
gz sim -r empty.sdf
```

Terminal 2 (ROS bridge):
```bash
source /opt/ros/humble/setup.bash
cd ~/digital_twin_ws
source install/setup.bash
python3 unity_bridge_demo.py
```

## Module Structure

The documentation will be organized as follows in `docs/module2/`:

```
docs/
└── module2/
    ├── index.mdx
    ├── gazebo-physics-modeling.mdx
    ├── sdf-vs-urdf-in-depth.mdx
    ├── sensor-simulation-lidar-depth.mdx
    └── unity-ros-high-fidelity-bridge.mdx
```

## Next Steps

1. Explore the full documentation in the `docs/module2/` directory
2. Review Gazebo physics modeling in `docs/module2/gazebo-physics-modeling.mdx`
3. Learn about SDF vs URDF in `docs/module2/sdf-vs-urdf-in-depth.mdx`
4. Understand sensor simulation in `docs/module2/sensor-simulation-lidar-depth.mdx`
5. Master Unity integration in `docs/module2/unity-ros-high-fidelity-bridge.mdx`

## Troubleshooting

### Common Issues:

1. **Gazebo won't start**
   - Solution: Check for proper ROS 2 Humble and Gazebo Harmonic installation

2. **Unity connection fails**
   - Solution: Verify that Unity is configured to connect to the correct IP and port

3. **Performance issues with synchronization**
   - Solution: Adjust simulation step sizes and update frequencies appropriately

4. **Python path issues**
   - Solution: Use virtual environments for Python development to avoid conflicts

## Performance Verification

To verify that your setup meets the performance requirements (100ms visualization, 10ms control):

```python
# performance_test.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import time

class PerformanceTester(Node):
    def __init__(self):
        super().__init__('performance_tester')
        self.publisher = self.create_publisher(Header, 'performance_test', 10)
        self.subscriber = self.create_subscription(
            Header,
            'performance_test',
            self.callback,
            10
        )
        self.times = []
        
    def callback(self, msg):
        receive_time = time.time()
        send_time = float(msg.stamp.sec) + float(msg.stamp.nanosec) / 1e9
        latency = (receive_time - send_time) * 1000  # latency in ms
        self.times.append(latency)
        
        # Print average latency every 10 messages
        if len(self.times) % 10 == 0:
            avg_latency = sum(self.times[-10:]) / 10
            max_latency = max(self.times[-10:])
            self.get_logger().info(f'Avg latency: {avg_latency:.2f}ms, Max latency: {max_latency:.2f}ms')
            
            # Check if latency requirements are met
            if avg_latency > 100:
                self.get_logger().warn(f'Average latency {avg_latency:.2f}ms exceeds 100ms requirement')
            else:
                self.get_logger().info('Latency requirements are satisfied')

def main():
    rclpy.init()
    tester = PerformanceTester()
    
    # Publish a message every 20ms to test 50Hz frequency
    timer = tester.create_timer(0.02, tester.publish_test_message)
    
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run this test to verify your system meets the performance requirements.