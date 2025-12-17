---
title: "Week 1 - ROS 2 Architecture and Fundamentals"
sidebar_label: "Week 1: ROS 2 Architecture"
description: "Understanding the architecture of ROS 2 and its role in robotic systems"
keywords: [ros2, architecture, middleware, dds, robotics, fundamentals]
---

# Week 1: ROS 2 Architecture and Fundamentals

## Learning Outcomes

By the end of this week, you will be able to:
- Explain the ROS 2 architecture and its layered design
- Identify the role of DDS (Data Distribution Service) in ROS 2
- Understand the concept of ROS 2 nodes and packages
- Recognize the advantages of ROS 2 over ROS 1
- Describe how ROS 2 supports real-time and distributed robotic systems

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is an open-source framework designed for building robotic applications. Despite its name, ROS is not an operating system but rather a collection of tools, libraries, and conventions that simplify the task of creating complex and robust robotic applications.

ROS 2 evolved from ROS 1 to address several key limitations, including:
- Real-time support
- Multi-robot systems
- Production deployment
- Security features
- Improved middleware flexibility

## ROS 2 Architecture Overview

ROS 2 follows a distributed architecture built on top of DDS (Data Distribution Service), a middleware standard for real-time systems. This architecture enables:

### Client Library Layer
- **rclcpp**: C++ client library
- **rclpy**: Python client library (focus of this curriculum)
- **rcljava**: Java client library
- **rclnodejs**: Node.js client library

### ROS Client Library (rcl)
- Provides consistent interface across languages
- Handles ROS-specific concepts (nodes, topics, services)
- Interfaces with middleware layer

### Middleware Layer (DDS)
- **Data Distribution Service**: Standard for real-time systems
- **Implementation options**: Fast DDS, Cyclone DDS, RTI Connext DDS
- Handles message routing, quality of service, discovery

### Platform Abstraction Layer
- System-level operations
- OS-specific implementations
- Hardware abstraction

## Key Concepts in ROS 2

### Nodes
Nodes are the fundamental execution units in ROS 2. Each node represents a single process that performs specific computation. Nodes can:
- Publish messages to topics
- Subscribe to topics
- Provide services
- Call services
- Send and receive actions

### Topics
Topics provide asynchronous, one-way communication between nodes using a publish-subscribe model:
- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can use the same topic
- Communication is decoupled in time and space

### Services
Services provide synchronous, request-response communication:
- Client sends request to server
- Server processes request and sends response
- Communication is blocking until response is received

### Actions
Actions provide goal-oriented communication with feedback:
- Long-running tasks with status updates
- Goal cancellation capability
- Feedback during execution
- Result upon completion

## Quality of Service (QoS) Profiles

ROS 2 introduces Quality of Service profiles to handle different communication requirements:

### Reliability
- **Reliable**: All messages delivered (with retries)
- **Best Effort**: Messages may be lost (higher performance)

### Durability
- **Transient Local**: Late-joining subscribers receive last message
- **Volatile**: No message persistence

### History
- **Keep Last**: Store only recent messages
- **Keep All**: Store all messages (memory intensive)

## DDS Implementation

DDS (Data Distribution Service) is the underlying middleware that enables ROS 2's distributed architecture:

### Discovery
- Automatic discovery of nodes, topics, and services
- Dynamic network formation
- No central master required

### Communication Patterns
- Publish-subscribe for topics
- Request-reply for services
- Client-server for actions

### Real-time Capabilities
- Predictable message delivery
- Configurable timing constraints
- Support for real-time operating systems

## ROS 2 vs ROS 1: Key Differences

### Middleware Architecture
- **ROS 1**: Single master architecture with TCPROS
- **ROS 2**: DDS-based decentralized architecture

### Real-time Support
- **ROS 1**: Limited real-time capabilities
- **ROS 2**: Designed with real-time systems in mind

### Multi-robot Support
- **ROS 1**: Complex to set up multi-robot systems
- **ROS 2**: Natural multi-robot communication

### Security
- **ROS 1**: Minimal built-in security
- **ROS 2**: Comprehensive security framework

### Language Support
- **ROS 1**: Primary focus on C++ and Python
- **ROS 2**: Extensible to multiple languages

## ROS 2 Ecosystem Components

### Development Tools
- **ros2 command**: Main command-line interface
- **rqt**: GUI tools for debugging and visualization
- **rviz**: 3D visualization tool
- **rosbag**: Data recording and playback

### Build System
- **colcon**: Modern build system
- **ament**: ROS 2's build and test infrastructure
- **CMake/Python setup**: Build configuration

### Package Management
- **rosdep**: System dependency manager
- **rosinstall_generator**: Source tree builder
- **vcs tools**: Version control management

## ROS 2 in Humanoid Robotics

ROS 2 is particularly well-suited for humanoid robotics applications:

### Distributed Architecture
- Separate nodes for different robot subsystems
- Coordination between perception, planning, and control
- Modular development and testing

### Real-time Performance
- Critical for balance and motion control
- Predictable message delivery
- Support for real-time operating systems

### Multi-robot Coordination
- Humanoid robots working with other robots
- Human-robot teaming scenarios
- Multi-robot systems in shared environments

## Hands-On Exercise

Research and compare two different DDS implementations (e.g., Fast DDS and Cyclone DDS). What are their respective advantages and disadvantages? In what scenarios would you choose one over the other for a humanoid robot application?

## Summary

ROS 2's architecture provides a robust foundation for developing complex robotic systems. Its DDS-based middleware supports distributed, real-time applications while providing familiar ROS concepts like nodes, topics, services, and actions. Understanding this architecture is essential for effectively developing humanoid robotics applications that can scale from single robots to complex multi-robot systems.