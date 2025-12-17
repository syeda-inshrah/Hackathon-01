---
title: "Week 2 - Nodes, Topics, Services, and Actions in ROS 2"
sidebar_label: "Week 2: ROS 2 Communication"
description: "Understanding ROS 2 communication patterns with practical examples of nodes, topics, services, and actions"
keywords: [ros2, nodes, topics, services, actions, communication, robotics, python]
---

# Week 2: Nodes, Topics, Services, and Actions in ROS 2

## Learning Outcomes

By the end of this week, you will be able to:
- Create ROS 2 nodes using Python and rclpy
- Implement publisher and subscriber patterns using topics
- Design request-response communication with services
- Build goal-oriented systems using actions
- Implement communication patterns for humanoid robot applications

## Understanding ROS 2 Nodes

A node is the fundamental computational element in ROS 2. Each node represents a single process that performs specific computation within the robotic system. Nodes are the containers for your robot's functionality.

### Node Structure

Every ROS 2 node includes:

```python title="Basic Node Structure"
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node_name')
        # Node initialization code here
        
def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

ROS 2 nodes follow a specific lifecycle:
- **Unconfigured**: Node created but not configured
- **Inactive**: Configuration applied, resources allocated
- **Active**: Full functionality available
- **Finalized**: Node destroyed, resources cleaned up

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, many-to-many communication using the publish-subscribe pattern. This is ideal for sensor data, robot state information, and other continuous data streams.

### Creating a Publisher Node

```python title="ROS 2 Publisher Example"
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
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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

### Creating a Subscriber Node

```python title="ROS 2 Subscriber Example"
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
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topic Communication Characteristics

- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Decoupled**: Publishers don't know subscribers exist
- **Many-to-many**: Multiple publishers and subscribers can use the same topic
- **Data-driven**: Communication occurs when data is available

## Services: Request-Response Communication

Services enable synchronous, bidirectional communication with request-response patterns. This is ideal for operations that require immediate results.

### Creating a Service Server

```python title="ROS 2 Service Server Example"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python title="ROS 2 Service Client Example"
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Communication Characteristics

- **Synchronous**: Client waits for response before continuing
- **Request-response**: One request generates one response
- **Stateless**: Each request is handled independently
- **Reliable**: Request guaranteed to be received (if server is available)

## Actions: Goal-Oriented Communication

Actions provide communication for long-running, goal-oriented tasks that require feedback and the ability to cancel.

### Creating an Action Server

```python title="ROS 2 Action Server Example"
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            self.get_logger().info('Publishing feedback: {0}'.format(
                feedback_msg.sequence))
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))
        
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Client

```python title="ROS 2 Action Client Example"
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

### Action Communication Characteristics

- **Long-running**: Designed for tasks that take time to complete
- **Feedback**: Continuous progress updates during execution
- **Cancelability**: Ability to cancel goals in progress
- **State tracking**: Goal state (pending, active, succeeded, etc.)

## Designing Communication Patterns for Humanoid Robots

### Sensor Data Distribution
- **Topics**: Sensor data streams (IMU, cameras, LiDAR, joint positions)
- **High frequency**: Multiple updates per second
- **Many subscribers**: Perception, control, monitoring nodes

### Control Commands
- **Services**: Configuration changes, mode switching
- **Actions**: Complex maneuvers, walking patterns
- **Topics**: Continuous command streams (joint trajectories)

### Coordination and Planning
- **Services**: Path planning requests
- **Actions**: Navigation goals with feedback
- **Topics**: Robot status and environment maps

## Communication Patterns Best Practices

### Topic Design
- Use intuitive names (e.g., `/sensor/imu/data`, `/robot/joint_states`)
- Choose appropriate message types
- Consider QoS settings for your application
- Avoid publishing large messages frequently

### Service Design
- Use services for operations with clear outcomes
- Keep services stateless when possible
- Handle errors gracefully
- Document service interfaces clearly

### Action Design
- Use actions for long-running, goal-oriented tasks
- Provide meaningful feedback messages
- Implement timeout handling
- Consider cancelability requirements

## Hands-On Exercise

Design a communication pattern for a humanoid robot picking up an object. Identify what nodes would be involved and what communication methods (topics, services, actions) would be appropriate for each interaction. Create simple Python nodes that demonstrate this pattern using rclpy.

## Summary

Understanding ROS 2's communication patterns is essential for building effective robotic applications. Topics provide asynchronous communication for data streams, services enable synchronous request-response patterns, and actions support long-running, goal-oriented tasks. Each communication type serves specific purposes in robotic systems, and selecting the right pattern is crucial for creating robust, efficient humanoid robot applications.