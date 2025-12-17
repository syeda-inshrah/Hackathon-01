# Unity Bridge Interface Contract

**Interface**: UnityBridgeInterface
**Implementation**: Python with ROS-TCP-Endpoint
**Version**: 1.0

## Purpose
This contract defines the interface for connecting Unity as a high-fidelity visualizer over Gazebo, focusing on communication bridge establishment and state synchronization.

## Methods

### establish_connection(host_ip, port, timeout)
Establishes a connection between Gazebo and Unity via ROS-TCP-Endpoint.

**Parameters**:
- host_ip (string): IP address of the Unity instance
- port (int): Port number for the connection
- timeout (float): Connection timeout in seconds

**Returns**: ConnectionHandle object with connection status and communication methods

**Errors**:
- ConnectionTimeoutError: If the connection cannot be established within the timeout period
- InvalidAddressError: If the IP address or port is invalid

### synchronize_robot_state(connection_handle, robot_state)
Synchronizes the robot state between Gazebo physics engine and Unity visualizer.

**Parameters**:
- connection_handle (ConnectionHandle): Handle to the established connection
- robot_state (DigitalTwinState): State object containing robot positions and sensor data

**Returns**: Boolean indicating successful synchronization

**Errors**:
- ConnectionLostError: If the connection is lost during synchronization
- InvalidStateError: If the robot state is invalid or malformed

### send_control_commands(connection_handle, commands_dict)
Sends control commands from Unity back to the Gazebo simulation.

**Parameters**:
- connection_handle (ConnectionHandle): Handle to the established connection
- commands_dict (dict): Dictionary mapping joint names to command values

**Returns**: Boolean indicating successful transmission

**Errors**:
- ConnectionLostError: If the connection is lost during transmission
- InvalidCommandsError: If the command values are out of valid ranges

### register_visualization_callback(connection_handle, callback_function)
Registers a callback function to handle visualization-specific events.

**Parameters**:
- connection_handle (ConnectionHandle): Handle to the established connection
- callback_function (function): Function to call when visualization events occur

**Returns**: CallbackHandle object for managing the callback

**Errors**:
- InvalidCallbackError: If the callback function is not callable

### get_visualization_data(connection_handle, data_type)
Retrieves visualization-specific data from the Unity instance.

**Parameters**:
- connection_handle (ConnectionHandle): Handle to the established connection
- data_type (string): Type of visualization data to retrieve

**Returns**: VisualizationData object containing the requested data

**Errors**:
- ConnectionLostError: If the connection is lost during data retrieval
- InvalidDataTypeError: If the data_type is not supported

## Performance Requirements
- Unity-Gazebo communication latency must be less than 100ms for visualization updates
- State synchronization should happen at minimum 30Hz for smooth visualization
- Control command transmission should have less than 50ms latency
- Connection must remain stable during extended simulation periods (>= 1 hour)