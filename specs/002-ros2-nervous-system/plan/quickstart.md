# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

**Feature**: Module 1: The Robotic Nervous System (ROS 2)
**Branch**: 002-ros2-nervous-system
**Created**: 2025-12-15

## Prerequisites

Before starting with this module, ensure you have:

- Ubuntu 22.04 LTS installed
- ROS 2 Humble Hawksbill installed with Python development tools
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
sudo apt install -y ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-action-tutorials-py
sudo apt install -y ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-gazebo-ros-pkgs

# Initialize rosdep
sudo rosdep init
rosdep update

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 2. Set up your Python workspace

```bash
# Create a workspace directory
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws

# Source ROS 2 environment each time you open a new terminal
source /opt/ros/humble/setup.bash

# Create a colcon workspace
colcon build
source install/setup.bash
```

### 3. Install Python dependencies

```bash
pip3 install rclpy urdf-parser-py transforms3d numpy matplotlib
pip3 install tensorflow torch  # For AI/ML agent examples
```

### 4. Set up Docusaurus documentation environment

```bash
npx create-docusaurus@latest docs-site classic
cd docs-site
npm install
```

## Hello World: Simple ROS 2 Node

Let's create a simple publisher/subscriber example to verify your environment:

### Create the publisher node:

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Create the subscriber node:

```python
# subscriber_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Run the example:

Terminal 1:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_humble_ws
source install/setup.bash
python3 publisher_member_function.py
```

Terminal 2:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_humble_ws
source install/setup.bash
python3 subscriber_member_function.py
```

## Next Steps

1. Explore the full documentation in the `docs/module1/` directory
2. Review ROS 2 fundamentals in `docs/module1/ros2-fundamentals.mdx`
3. Learn about communication patterns in `docs/module1/ros2-communication-patterns.mdx`
4. Understand the Python bridge in `docs/module1/python-rclpy-bridge.mdx`
5. Master URDF for humanoids in `docs/module1/urdf-for-humanoids.mdx`

## Troubleshooting

### Common Issues:

1. **ModuleNotFoundError: No module named 'rclpy'**
   - Solution: Ensure you've installed ROS 2 Humble and sourced the environment: `source /opt/ros/humble/setup.bash`

2. **ROS_DOMAIN_ID conflicts**
   - Solution: Set a unique domain ID: `export ROS_DOMAIN_ID=your_unique_id` (0-101)

3. **Python path issues**
   - Solution: Use virtual environments for Python development to avoid conflicts

## Performance Verification

To verify that your setup meets the performance requirements (100ms latency, 50Hz control loop):

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
            self.get_logger().info(f'Average latency over last 10 messages: {avg_latency:.2f}ms')
            
    def publish_test_message(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

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