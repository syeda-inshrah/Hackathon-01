# ROS 2 Node Interface Contract

**Interface**: ROS2Node
**Implementation**: Python with rclpy
**Version**: 1.0

## Purpose
This contract defines the interface for creating ROS 2 nodes in Python that can publish and subscribe to topics with standard message types.

## Methods

### create_publisher(topic_name, message_type, qos_profile=None)
Creates a publisher for a specific topic.

**Parameters**:
- topic_name (string): Name of the topic to publish to
- message_type (class): ROS message type (e.g., std_msgs.msg.String)
- qos_profile (QoSProfile, optional): Quality of service settings

**Returns**: Publisher object

**Errors**:
- InvalidTopicNameError: If topic_name doesn't follow ROS naming conventions
- InvalidMessageTypeError: If message_type is not a valid ROS message type

### create_subscriber(topic_name, message_type, callback, qos_profile=None)
Creates a subscriber for a specific topic.

**Parameters**:
- topic_name (string): Name of the topic to subscribe to
- message_type (class): ROS message type (e.g., std_msgs.msg.String)
- callback (function): Function to call when message is received
- qos_profile (QoSProfile, optional): Quality of service settings

**Returns**: Subscriber object

**Errors**:
- InvalidTopicNameError: If topic_name doesn't follow ROS naming conventions
- InvalidMessageTypeError: If message_type is not a valid ROS message type
- InvalidCallbackError: If callback is not a callable function

### create_service(service_name, service_type, callback)
Creates a service server.

**Parameters**:
- service_name (string): Name of the service
- service_type (class): ROS service type (e.g., std_srvs.srv.SetBool)
- callback (function): Function to handle service requests

**Returns**: Service server object

**Errors**:
- InvalidServiceNameError: If service_name doesn't follow ROS naming conventions
- InvalidServiceTypeError: If service_type is not a valid ROS service type
- InvalidCallbackError: If callback is not a callable function

### create_client(service_name, service_type)
Creates a service client.

**Parameters**:
- service_name (string): Name of the service to call
- service_type (class): ROS service type (e.g., std_srvs.srv.SetBool)

**Returns**: Service client object

**Errors**:
- InvalidServiceNameError: If service_name doesn't follow ROS naming conventions
- InvalidServiceTypeError: If service_type is not a valid ROS service type

### create_action_server(action_name, action_type, goal_callback, execute_callback, cancel_callback)
Creates an action server.

**Parameters**:
- action_name (string): Name of the action
- action_type (class): ROS action type (e.g., rclpy.action.Fibonacci)
- goal_callback (function): Function to handle goal requests
- execute_callback (function): Function to execute the action
- cancel_callback (function): Function to handle cancellation requests

**Returns**: Action server object

**Errors**:
- InvalidActionNameError: If action_name doesn't follow ROS naming conventions
- InvalidActionTypeError: If action_type is not a valid ROS action type

### create_action_client(action_name, action_type)
Creates an action client.

**Parameters**:
- action_name (string): Name of the action to call
- action_type (class): ROS action type (e.g., rclpy.action.Fibonacci)

**Returns**: Action client object

**Errors**:
- InvalidActionNameError: If action_name doesn't follow ROS naming conventions
- InvalidActionTypeError: If action_type is not a valid ROS action type

## Performance Requirements
- All operations must complete within 100ms
- The node must support a control loop frequency of minimum 50Hz