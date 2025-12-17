---
title: "Week 3 - VSLAM and Nav2 Integration for Humanoid Robots"
sidebar_label: "Week 3: VSLAM and Navigation"
description: "Understanding Visual SLAM and Nav2 integration with Isaac for humanoid robot navigation"
keywords: [vslam, nav2, navigation, localization, mapping, robotics, humanoid]
---

# Week 3: VSLAM and Nav2 Integration for Humanoid Robots

## Learning Outcomes

By the end of this week, you will be able to:
- Integrate Visual SLAM with Nav2 navigation stack
- Configure navigation for humanoid robot specific requirements
- Implement VSLAM-based localization in Nav2
- Design navigation behaviors suitable for humanoid robots
- Optimize navigation for human environments and bipedal locomotion

## Introduction to Visual SLAM and Navigation Integration

Visual SLAM (Simultaneous Localization and Mapping) and navigation represent two complementary technologies in robotics. SLAM enables robots to build maps of unknown environments and determine their location within these maps, while navigation uses these maps to plan and execute paths from start to goal locations.

For humanoid robots, these technologies are particularly important because they need to operate in human environments that were designed for human navigation patterns and capabilities.

## Visual SLAM in Navigation Context

### Mapping vs Localization

Visual SLAM performs two key functions that directly impact navigation:

1. **Mapping**: Creating a representation of the environment
2. **Localization**: Determining the robot's position within the environment

### SLAM for Navigation Requirements

For effective navigation, SLAM systems need to provide:

- **Metric Accuracy**: Precise measurements for path planning
- **Consistency**: Stable map representations over time
- **Real-time Updates**: Adaptation to dynamic environments
- **Semantic Information**: Understanding of traversable areas

### Visual SLAM Challenges for Navigation

- **Feature-poor Environments**: Corridors, large rooms with few features
- **Dynamic Objects**: Moving objects that can confuse SLAM
- **Lighting Changes**: Different visual appearance during day/night
- **Robot Motion**: Head movements during walking that affect visual odometry

## Nav2 Navigation Stack Overview

Nav2 (Navigation 2) is the ROS 2 navigation stack that provides path planning, obstacle avoidance, and locomotion for mobile robots.

### Core Nav2 Components

- **Global Planner**: Computes optimal path from start to goal
- **Local Planner**: Executes path while avoiding obstacles
- **Controller**: Translates navigation commands to robot actions
- **Behavior Trees**: Orchestrates navigation task execution
- **Lifecycle Manager**: Manages state of navigation components

### Nav2 for Humanoid Robots

Nav2 requires specific configuration for humanoid robots due to their unique characteristics:

- **Bipedal Locomotion**: Different movement constraints than wheeled robots
- **Balance Requirements**: Need to maintain balance during navigation
- **Human-Scale Environments**: Designed for human navigation patterns
- **Social Navigation**: Need to navigate respectfully around humans

## Integration Architecture

### SLAM and Navigation System Architecture

```
Camera(s) → Image Processing → Visual SLAM → Map + Pose
                                        ↓
                              Navigation Stack → Path Planning
                                        ↓
                              Controller → Robot Motion
```

### TF Frame Considerations

For humanoid robots, the transform tree becomes more complex:

```
map
 └── odom
     └── base_footprint (for bipedal navigation)
         └── base_link
             ├── head_link
             ├── torso_link
             ├── left_leg_link
             └── right_leg_link
```

### Data Flow Between SLAM and Navigation

1. **Initialization Phase**: SLAM creates initial map
2. **Localization Phase**: SLAM provides pose relative to map
3. **Navigation Phase**: Navigation uses map and pose for path planning
4. **Map Update Phase**: SLAM updates map as robot moves
5. **Relocalization Phase**: SLAM recovers if localization fails

## Configuring Nav2 for Humanoid Robots

### Navigation Parameters for Humanoid Locomotion

```yaml title="nav2 params for humanoid robot"
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Behavior tree for humanoid navigation
    default_nav_through_poses_bt_xml: humanoid_navigate_through_poses_w_replanning_and_recovery.xml
    default_nav_to_pose_bt_xml: humanoid_navigate_to_pose_w_replanning_and_recovery.xml

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    # Humanoid-specific controller
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.15
      wz_std: 0.3
      vx_max: 0.3
      vx_min: -0.15
      vy_max: 0.15
      wz_max: 0.3
      sim_frequency: 20.0
      control_frequency: 20.0
      transform_tolerance: 0.1
      # Humanoid-specific parameters
      max_acceleration: 0.5  # Human-like acceleration limits
      max_deceleration: 0.8  # More aggressive stopping for safety

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_rollout_costs: true
      always_send_full_costmap: true
      width: 4
      height: 4
      resolution: 0.05  # Higher resolution for precise humanoid navigation
      origin_x: -2.0
      origin_y: -2.0
      # Foot print for bipedal robot
      footprint: "[[-0.15, -0.1], [-0.15, 0.1], [0.15, 0.1], [0.15, -0.1]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5  # Appropriate for humanoid walking
        cost_scaling_factor: 3.0
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 2.0  # Consider obstacles up to human height
          clearing: True
          marking: True
          data_type: "LaserScan"

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: false
      width: 40
      height: 40
      resolution: 0.05  # Higher resolution for detailed planning
      origin_x: -20.0
      origin_y: -20.0
      # Humanoid-specific global map
      footprint: "[[-0.15, -0.1], [-0.15, 0.1], [0.15, 0.1], [0.15, -0.1]]"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        voxel_size: 0.1
        max_obstacle_height: 2.0
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.8  # Consider safety margin for humanoid
```

### Humanoid-Specific Navigation Behaviors

```python title="Humanoid Navigation Behavior Implementation"
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class HumanoidNavigationNode(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_node')
        
        # Create action client for navigation
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # Store robot state
        self.current_pose = None
        self.is_moving = False
        self.balance_threshold = 0.1  # Balance error threshold
        
        # Initialize humanoid-specific parameters
        self.step_size = 0.3  # Max step size for humanoid
        self.turn_rate = 0.2  # Conservative turning rate
        self.safety_distance = 0.5  # Safety distance from obstacles
        
        self.get_logger().info('Humanoid Navigation node initialized')

    def navigate_to_pose(self, x, y, theta):
        """Navigate to specified pose with humanoid-specific constraints"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Check if goal is reachable and safe for humanoid
        if not self.is_safe_goal(goal.pose.pose):
            self.get_logger().error('Navigation goal is not safe for humanoid')
            return False

        # Wait for navigation server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Send navigation goal
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.navigation_result_callback)
        
        return True

    def is_safe_goal(self, pose):
        """Check if navigation goal is safe for humanoid robot"""
        # Check distance from current position
        if self.current_pose:
            dist = math.sqrt(
                (pose.position.x - self.current_pose.position.x)**2 +
                (pose.position.y - self.current_pose.position.y)**2
            )
            
            # Ensure goal is not too far
            if dist > 50.0:  # Max navigation distance
                return False
        
        # Check for potential obstacles in path
        # (This would integrate with costmap in a full implementation)
        
        return True

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        goal_result = future.result()
        
        if goal_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Navigation failed with status: %d' % goal_result.status)
            return
        
        self.get_logger().info('Navigation completed successfully')
        self.is_moving = False

    def check_balance(self):
        """Check if robot is maintaining balance during navigation"""
        # In a real implementation, this would interface with balance control
        # Return True if balance is maintained, False otherwise
        # For simulation, we'll return True
        return True

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNavigationNode()
    
    # Example: Navigate to a specific pose
    node.navigate_to_pose(1.0, 2.0, 0.0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## VSLAM-Based Localization in Nav2

### AMCL vs VSLAM Localization

Traditional Nav2 uses AMCL (Adaptive Monte Carlo Localization) for position estimation in a known map. VSLAM provides an alternative that doesn't require a pre-built map.

### Integrating VSLAM with Nav2

The integration typically involves:

1. **SLAM Node**: Creates map and provides pose
2. **Map Server**: Provides map to Nav2 components
3. **Transformations**: Proper TF tree setup
4. **Coordinate Frames**: Consistent frame definitions

### Configuration for VSLAM-Integrated Navigation

```python title="VSLAM Integration Node"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np

class VSLAMNavIntegrationNode(Node):
    def __init__(self):
        super().__init__('vslam_nav_integration_node')
        
        # SLAM pose input
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_pose',
            self.slam_pose_callback,
            10
        )
        
        # Create publisher for navigation pose
        self.nav_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            10
        )
        
        # Create publisher for odometry (for navigation)
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        # Transform broadcaster for navigation
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store pose history for smoother navigation
        self.pose_history = []
        self.max_history = 10
        
        self.get_logger().info('VSLAM-Nav integration node initialized')
    
    def slam_pose_callback(self, msg):
        """Process SLAM pose and publish for navigation"""
        # Update pose history
        self.pose_history.append(msg.pose)
        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)
        
        # Calculate smoothed pose (optional)
        smoothed_pose = self.smooth_pose(msg.pose)
        
        # Publish pose for navigation (acts as AMCL replacement)
        nav_pose_msg = PoseWithCovarianceStamped()
        nav_pose_msg.header = msg.header
        nav_pose_msg.header.frame_id = 'map'
        nav_pose_msg.pose = smoothed_pose
        self.nav_pose_pub.publish(nav_pose_msg)
        
        # Publish odometry for navigation
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose = smoothed_pose
        self.odom_pub.publish(odom_msg)
        
        # Broadcast transform for navigation
        self.broadcast_transforms(smoothed_pose, msg.header.stamp)
    
    def smooth_pose(self, pose):
        """Apply smoothing to reduce pose jitter for navigation"""
        if len(self.pose_history) == 0:
            return pose
        
        # Calculate average position from history
        avg_x = sum([p.pose.position.x for p in self.pose_history]) / len(self.pose_history)
        avg_y = sum([p.pose.position.y for p in self.pose_history]) / len(self.pose_history)
        avg_z = sum([p.pose.position.z for p in self.pose_history]) / len(self.pose_history)
        
        # Use latest orientation (averaging quaternions is complex)
        last_orientation = self.pose_history[-1].pose.orientation
        
        smoothed_pose = pose
        smoothed_pose.pose.position.x = avg_x
        smoothed_pose.pose.position.y = avg_y
        smoothed_pose.pose.position.z = avg_z
        smoothed_pose.pose.orientation = last_orientation
        
        return smoothed_pose
    
    def broadcast_transforms(self, pose, stamp):
        """Broadcast necessary transforms for navigation"""
        # Transform from map to odom (SLAM -> Nav2)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = pose.pose.position.x
        t_map_odom.transform.translation.y = pose.pose.position.y
        t_map_odom.transform.translation.z = pose.pose.position.z
        t_map_odom.transform.rotation = pose.pose.orientation
        
        # Transform from odom to base_footprint
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_footprint'
        # This would typically come from robot's odometry/kinematics
        
        self.tf_broadcaster.sendTransform([t_map_odom, t_odom_base])

def main(args=None):
    rclpy.init(args=args)
    node = VSLAMNavIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bipedal Navigation Challenges

### Balance and Stability

Humanoid robots face unique challenges during navigation:

- **Dynamic Balance**: Maintaining balance while moving
- **Step Planning**: Planning stable footstep sequences
- **Terrain Adaptation**: Adapting to uneven surfaces
- **Recovery**: Handling balance disturbances

### Navigation Parameters for Bipedal Robots

- **Step Size Limits**: Maximum distance per step
- **Turning Constraints**: Limited turning radius
- **Speed Limitations**: Slower than wheeled robots
- **Stability Requirements**: More conservative obstacle avoidance

## Hands-On Exercise

1. Set up a simulation environment with a humanoid robot model
2. Configure Visual SLAM for the robot using Isaac Sim or Gazebo
3. Integrate the SLAM system with Nav2 for navigation
4. Test navigation in different environments (corridor, open space, cluttered area)
5. Implement a simple bipedal-specific navigation behavior

## Summary

Integrating Visual SLAM with Nav2 navigation creates a powerful system for humanoid robots operating in unknown environments. The combination allows robots to simultaneously explore and map unknown spaces while navigating safely to goals. For humanoid robots, special consideration must be given to balance constraints, step planning, and navigation behaviors that account for bipedal locomotion. The successful integration of these systems enables humanoid robots to autonomously navigate complex human environments, a critical capability for practical deployment.