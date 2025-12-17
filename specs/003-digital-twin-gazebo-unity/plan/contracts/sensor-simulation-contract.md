# Sensor Simulation Interface Contract

**Interface**: SensorSimulationInterface
**Implementation**: Python with gazebo_ros_pkgs
**Version**: 1.0

## Purpose
This contract defines the interface for simulating LiDAR and depth camera sensors in Gazebo, including realistic noise models and ROS 2 topic bridging.

## Methods

### create_lidar_sensor(sensor_name, parent_link, position, orientation, range_min, range_max, resolution, noise_params)
Creates and configures a simulated LiDAR sensor in Gazebo.

**Parameters**:
- sensor_name (string): Name of the sensor
- parent_link (string): Name of the link to attach the sensor to
- position (geometry_msgs.Point): Position of the sensor relative to the parent link
- orientation (geometry_msgs.Quaternion): Orientation of the sensor relative to the parent link
- range_min (float): Minimum sensing range (meters)
- range_max (float): Maximum sensing range (meters)
- resolution (float): Angular resolution (degrees)
- noise_params (dict): Parameters for the noise model

**Returns**: SensorHandle object with access to sensor data and configuration

**Errors**:
- InvalidParentLinkError: If the parent_link doesn't exist
- InvalidSensorParametersError: If sensor parameters are out of valid range

### create_depth_camera_sensor(sensor_name, parent_link, resolution_width, resolution_height, fov_horizontal, fov_vertical, noise_params)
Creates and configures a simulated depth camera sensor in Gazebo.

**Parameters**:
- sensor_name (string): Name of the sensor
- parent_link (string): Name of the link to attach the sensor to
- resolution_width (int): Width of the camera image in pixels
- resolution_height (int): Height of the camera image in pixels
- fov_horizontal (float): Horizontal field of view (degrees)
- fov_vertical (float): Vertical field of view (degrees)
- noise_params (dict): Parameters for the noise model

**Returns**: SensorHandle object with access to camera data and configuration

**Errors**:
- InvalidParentLinkError: If the parent_link doesn't exist
- InvalidResolutionError: If resolution values are out of valid range
- InvalidFOVError: If field of view values are invalid

### publish_point_cloud(sensor_handle, topic_name)
Starts publishing point cloud data from the LiDAR sensor to the specified ROS 2 topic.

**Parameters**:
- sensor_handle (SensorHandle): Handle to the LiDAR sensor
- topic_name (string): Name of the ROS 2 topic to publish to

**Returns**: Boolean indicating success

**Errors**:
- InvalidSensorHandleError: If the sensor handle is invalid
- InvalidTopicNameError: If the topic name is invalid

### publish_depth_image(sensor_handle, topic_name)
Starts publishing depth image data from the depth camera sensor to the specified ROS 2 topic.

**Parameters**:
- sensor_handle (SensorHandle): Handle to the depth camera sensor
- topic_name (string): Name of the ROS 2 topic to publish to

**Returns**: Boolean indicating success

**Errors**:
- InvalidSensorHandleError: If the sensor handle is invalid
- InvalidTopicNameError: If the topic name is invalid

### get_sensor_data(sensor_handle)
Retrieves the latest sensor data from the specified sensor.

**Parameters**:
- sensor_handle (SensorHandle): Handle to the sensor

**Returns**: SensorData object containing the latest sensor readings

**Errors**:
- InvalidSensorHandleError: If the sensor handle is invalid
- SensorNotReadyError: If the sensor is not yet ready

## Performance Requirements
- Point cloud data must be published at minimum 10Hz for LiDAR
- Depth image data must be published at minimum 15Hz for depth cameras
- All sensor data access methods must complete within 10ms
- Noise model computation should not exceed 2ms per sensor update