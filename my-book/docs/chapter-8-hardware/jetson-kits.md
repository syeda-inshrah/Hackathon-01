---
title: "Jetson Kits for Physical AI Deployment"
sidebar_label: "Jetson Kits Setup"
description: "Setting up NVIDIA Jetson platforms for Physical AI and robotic deployment"
keywords: [jetson, orin, nano, nx, robotics, edge ai, deployment, physical ai]
---

# Jetson Kits for Physical AI Deployment

## Overview

For deploying Physical AI applications to real robots, edge computing platforms are essential. NVIDIA's Jetson family provides powerful, power-efficient AI computers designed for robotics and edge AI applications. This chapter covers setting up Jetson platforms for Physical AI deployment.

## Jetson Platform Overview

### Jetson Orin Series
The Jetson Orin series offers high performance for edge AI applications:

#### Jetson Orin Nano
- **AI Performance**: Up to 40 TOPS (trillions of operations per second)
- **GPU**: 1024-core NVIDIA Ampere architecture GPU
- **CPU**: 4-core ARM Cortex-A78AE v8.2 64-bit
- **Memory**: 4GB or 8GB LPDDR5
- **Power**: 7W to 15W
- **Use Case**: Light perception and control tasks

#### Jetson Orin NX
- **AI Performance**: Up to 100 TOPS
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **CPU**: 6-core ARM Cortex-A78AE v8.2 64-bit
- **Memory**: 8GB LPDDR5
- **Power**: 15W to 25W
- **Use Case**: Complex perception and planning

#### Jetson AGX Orin
- **AI Performance**: Up to 275 TOPS
- **GPU**: 2048-core NVIDIA Ampere architecture GPU
- **Memory**: 16GB or 32GB LPDDR5
- **Power**: 15W to 60W
- **Use Case**: Full autonomy, multiple sensors

### Platform Comparison

| Feature | Orin Nano (8GB) | Orin NX | AGX Orin (32GB) |
|---------|----------------|---------|-----------------|
| AI Performance | 40 TOPS | 100 TOPS | 275 TOPS |
| GPU Cores | 1024 | 2048 | 2048 |
| CPU Cores | 4 | 6 | 12 |
| Memory | 8GB LPDDR5 | 8GB LPDDR5 | 32GB LPDDR5 |
| Power | 7W-15W | 15W-25W | 15W-60W |
| Price | ~$359 | ~$499 | ~$1,099 |

## Setting Up Jetson Platform

### Initial Setup

#### 1. Hardware Requirements
- Jetson development kit
- Power adapter (official recommended)
- Micro-USB or USB-C cable for flashing
- HDMI monitor and USB keyboard/mouse (initial setup)
- Network connection (Ethernet recommended)

#### 2. Flashing the OS
The Jetson uses NVIDIA's SDK Manager for initial OS flashing:

```bash
# On host computer, install SDK Manager
# This is done on your development workstation
# Download from NVIDIA developer website

# Or use command line tool for headless setup
sudo apt install nvidia-jetpack-manager
```

#### 3. System Configuration

```python title="Jetson Configuration Script"
import subprocess
import os
import sys

class JetsonConfig:
    def __init__(self):
        self.jetson_model = self.detect_jetson_model()
        print(f"Detected Jetson model: {self.jetson_model}")
    
    def detect_jetson_model(self):
        """Detect the Jetson model"""
        try:
            with open('/proc/device-tree/nvidia,tegra-family', 'r') as f:
                family = f.read().strip().decode('utf-8', errors='ignore')
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip().decode('utf-8', errors='ignore')
            return f"{family} - {model}"
        except:
            return "Unknown Jetson Model"
    
    def install_jetpack_dependencies(self):
        """Install essential JetPack components"""
        print("Installing JetPack dependencies...")
        
        packages = [
            'nvidia-jetpack',  # Base JetPack
            'nvidia-jetpack-runtime',  # Runtime libraries
            'nvidia-container-runtime',  # Docker support
            'cuda-toolkit-11-4',  # CUDA toolkit
            'nvidia-opencv',  # OpenCV with CUDA
            'nvidia-isaac',  # Isaac SDK components
            'nvidia-vpi',  # Vision Programming Interface
            'nvidia-deepstream',  # Video analytics
        ]
        
        for package in packages:
            try:
                result = subprocess.run(['sudo', 'apt', 'install', '-y', package], 
                                      capture_output=True, text=True, timeout=300)
                if result.returncode == 0:
                    print(f"✅ Installed {package}")
                else:
                    print(f"❌ Failed to install {package}: {result.stderr}")
            except subprocess.TimeoutExpired:
                print(f"❌ Timeout installing {package}")
            except Exception as e:
                print(f"❌ Error installing {package}: {e}")
    
    def setup_ros2_environment(self):
        """Setup ROS 2 for robotics applications"""
        print("Setting up ROS 2 environment...")
        
        # Add ROS 2 repository
        subprocess.run(['sudo', 'apt', 'update'])
        subprocess.run(['sudo', 'apt', 'install', '-y', 'software-properties-common'])
        subprocess.run(['sudo', 'add-apt-repository', 'universe'])
        
        # Install ROS 2 base
        subprocess.run(['sudo', 'apt', 'install', '-y', 'ros-humble-ros-base'])
        
        # Install robot-specific packages
        robot_packages = [
            'ros-humble-navigation2',
            'ros-humble-nav2-bringup',
            'ros-humble-moveit',
            'ros-humble-rosbridge-suite',
            'ros-humble-openni2-launch',
            'ros-humble-usb-cam',
            'ros-humble-pointcloud-to-laserscan',
        ]
        
        for pkg in robot_packages:
            subprocess.run(['sudo', 'apt', 'install', '-y', pkg])
        
        print("✅ ROS 2 environment configured")
    
    def setup_docker_environment(self):
        """Setup Docker for containerized AI applications"""
        print("Setting up Docker environment...")
        
        # Install Docker
        subprocess.run(['sudo', 'apt', 'install', '-y', 'docker.io'])
        
        # Install NVIDIA Container Toolkit
        subprocess.run(['curl', '-sL', 'https://nvidia.github.io/nvidia-docker/gpgkey', '|', 'sudo', 'apt-key', 'add', '-'])
        
        # Add repository
        dist = subprocess.run(['. /etc/os-release && echo $UBUNTU_CODENAME'], 
                             capture_output=True, text=True, shell=True)
        subprocess.run(['curl', '-sL', f'https://nvidia.github.io/nvidia-docker/ubuntu{dist.stdout.strip()}/nvidia-docker.list', 
                       '|', 'sudo', 'tee', f'/etc/apt/sources.list.d/nvidia-docker.list'])
        
        subprocess.run(['sudo', 'apt', 'update'])
        subprocess.run(['sudo', 'apt', 'install', '-y', 'nvidia-container-toolkit'])
        
        # Restart Docker
        subprocess.run(['sudo', 'systemctl', 'restart', 'docker'])
        
        # Add user to docker group
        subprocess.run(['sudo', 'usermod', '-aG', 'docker', os.getlogin()])
        
        print("✅ Docker environment configured")
    
    def optimize_for_ai_workloads(self):
        """Apply optimizations for AI workloads"""
        print("Optimizing for AI workloads...")
        
        # Enable maximum performance mode
        try:
            subprocess.run(['sudo', 'nvpmodel', '-m', '0'], timeout=10)  # Maximum performance
            print("✅ Performance mode set to maximum")
        except:
            print("⚠️  Could not set performance mode (nvpmodel may not be available)")
        
        # Set fan to maximum performance (if applicable)
        try:
            subprocess.run(['sudo', 'jetson_clocks'], timeout=10)
            print("✅ Jetson clocks locked for maximum performance")
        except:
            print("⚠️  Could not lock jetson clocks")
        
        # Configure swap for memory-intensive tasks
        if self.check_swap_space() < 8:  # Less than 8GB
            self.configure_swap(8)  # Add 8GB swap
    
    def check_swap_space(self):
        """Check current swap space in GB"""
        try:
            result = subprocess.run(['free', '-g'], capture_output=True, text=True)
            lines = result.stdout.split('\n')
            for line in lines:
                if 'Swap:' in line:
                    parts = line.split()
                    if len(parts) >= 3:
                        return int(parts[2])  # Used swap in GB
        except:
            pass
        return 0
    
    def configure_swap(self, size_gb):
        """Configure swap space"""
        print(f"Configuring {size_gb}GB swap space...")
        
        # Create swap file
        subprocess.run(['sudo', 'fallocate', '-l', f'{size_gb}G', '/mnt/swapfile'])
        subprocess.run(['sudo', 'chmod', '600', '/mnt/swapfile'])
        subprocess.run(['sudo', 'mkswap', '/mnt/swapfile'])
        subprocess.run(['sudo', 'swapon', '/mnt/swapfile'])
        
        # Make persistent
        with open('/etc/fstab', 'a') as f:
            f.write('/mnt/swapfile swap swap defaults 0 0\n')
        
        print(f"✅ {size_gb}GB swap space configured")

def main():
    print("=== Jetson Configuration for Physical AI ===")
    
    config = JetsonConfig()
    
    print("\n1. Installing JetPack dependencies...")
    config.install_jetpack_dependencies()
    
    print("\n2. Setting up ROS 2 environment...")
    config.setup_ros2_environment()
    
    print("\n3. Setting up Docker environment...")
    config.setup_docker_environment()
    
    print("\n4. Optimizing for AI workloads...")
    config.optimize_for_ai_workloads()
    
    print("\n=== Configuration Complete ===")
    print("Please reboot the system to apply all changes:")
    print("  sudo reboot")

if __name__ == "__main__":
    main()
```

## Sensor Integration

### RealSense D435i Setup

The Intel RealSense D435i is a popular choice for robotic perception:

```python title="RealSense Integration on Jetson"
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCameras:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        
        # Create align object for depth-color alignment
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        print("RealSense camera initialized")
    
    def get_frames(self):
        """Get aligned depth and color frames"""
        frames = self.pipeline.wait_for_frames()
        
        # Align depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None, None
        
        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        return depth_image, color_image
    
    def depth_to_pointcloud(self, depth_frame, color_frame):
        """Convert depth image to point cloud"""
        # Get camera intrinsic parameters
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        
        # Create point cloud
        points = rs.pointcloud()
        points.map_to(color_frame)
        
        # Generate point cloud
        point_cloud = points.calculate(depth_frame)
        
        return point_cloud
    
    def get_depth_at_pixel(self, x, y):
        """Get depth value at specific pixel location"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if depth_frame:
            distance = depth_frame.get_distance(x, y)
            return distance
        return 0.0
    
    def __del__(self):
        self.pipeline.stop()

# Example usage
def main():
    camera = RealSenseCameras()
    
    try:
        while True:
            depth_image, color_image = camera.get_frames()
            
            if depth_image is not None and color_image is not None:
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                # Stack both images horizontally
                images = np.hstack((color_image, depth_colormap))
                
                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q or ESC to quit
                    break
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

### IMU Integration

Integrate IMU sensors for balance and orientation:

```python title="IMU Integration for Balance Control"
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import smbus
import time
import math

class JetsonIMU:
    def __init__(self, bus_num=1, device_addr=0x68):
        # Initialize I2C communication
        self.bus = smbus.SMBus(bus_num)
        self.device_addr = device_addr
        
        # Initialize the IMU (example for MPU6050)
        self.bus.write_byte_data(self.device_addr, 0x6B, 0x00)  # Wake up device
        
        # ROS publisher
        rospy.init_node('jetson_imu_publisher', anonymous=True)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        
        # Calibration values
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 0.0]
        
        print("IMU initialized on Jetson platform")
    
    def read_raw_data(self, addr):
        """Read raw 16-bit data from IMU"""
        high = self.bus.read_byte_data(self.device_addr, addr)
        low = self.bus.read_byte_data(self.device_addr, addr+1)
        value = ((high << 8) | low)
        
        # Handle negative values (two's complement)
        if value > 32768:
            value = value - 65536
        return value
    
    def get_imu_data(self):
        """Get IMU data as dictionary"""
        # Read accelerometer data
        acc_x = self.read_raw_data(0x3B)
        acc_y = self.read_raw_data(0x3D)
        acc_z = self.read_raw_data(0x3F)
        
        # Read gyroscope data
        gyro_x = self.read_raw_data(0x43)
        gyro_y = self.read_raw_data(0x45)
        gyro_z = self.read_raw_data(0x47)
        
        # Convert to physical units (this is simplified - see MPU6050 datasheet for exact conversion)
        acc_x_scaled = acc_x / 16384.0  # Assuming ±2g range
        acc_y_scaled = acc_y / 16384.0
        acc_z_scaled = acc_z / 16384.0
        
        gyro_x_scaled = gyro_x / 131.0   # Assuming ±250°/s range
        gyro_y_scaled = gyro_y / 131.0
        gyro_z_scaled = gyro_z / 131.0
        
        # Apply offsets (calibration)
        acc_x_scaled -= self.accel_offset[0]
        acc_y_scaled -= self.accel_offset[1]
        acc_z_scaled -= self.accel_offset[2]
        
        gyro_x_scaled -= self.gyro_offset[0]
        gyro_y_scaled -= self.gyro_offset[1]
        gyro_z_scaled -= self.gyro_offset[2]
        
        return {
            'acceleration': {'x': acc_x_scaled, 'y': acc_y_scaled, 'z': acc_z_scaled},
            'angular_velocity': {'x': gyro_x_scaled, 'y': gyro_y_scaled, 'z': gyro_z_scaled}
        }
    
    def calibrate_sensor(self, samples=100):
        """Calibrate the IMU sensor"""
        print("Starting IMU calibration...")
        
        # Collect samples for offset calculation
        gyro_sum = [0.0, 0.0, 0.0]
        accel_sum = [0.0, 0.0, 0.0]
        
        for i in range(samples):
            raw_data = self.get_imu_data()
            
            gyro_sum[0] += raw_data['angular_velocity']['x']
            gyro_sum[1] += raw_data['angular_velocity']['y']
            gyro_sum[2] += raw_data['angular_velocity']['z']
            
            accel_sum[0] += raw_data['acceleration']['x']
            accel_sum[1] += raw_data['acceleration']['y']
            accel_sum[2] += raw_data['acceleration']['z'] - 1.0  # Subtract gravity
            
            time.sleep(0.01)
        
        # Calculate offsets
        self.gyro_offset = [g / samples for g in gyro_sum]
        self.accel_offset = [a / samples for a in accel_sum]
        
        print(f"Gyro offsets: {self.gyro_offset}")
        print(f"Accel offsets: {self.accel_offset}")
        print("Calibration complete!")
    
    def run_publisher(self):
        """Run the IMU data publisher"""
        rate = rospy.Rate(100)  # 100 Hz
        
        while not rospy.is_shutdown():
            # Get IMU data
            imu_data = self.get_imu_data()
            
            # Create ROS IMU message
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            
            # Set angular velocity
            imu_msg.angular_velocity.x = imu_data['angular_velocity']['x']
            imu_msg.angular_velocity.y = imu_data['angular_velocity']['y']
            imu_msg.angular_velocity.z = imu_data['angular_velocity']['z']
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = imu_data['acceleration']['x']
            imu_msg.linear_acceleration.y = imu_data['acceleration']['y']
            imu_msg.linear_acceleration.z = imu_data['acceleration']['z']
            
            # Publish the message
            self.imu_pub.publish(imu_msg)
            
            rate.sleep()

# ROS node example
def imu_node():
    imu = JetsonIMU()
    
    # Calibrate sensor (optional, do this once)
    imu.calibrate_sensor()
    
    # Run publisher
    imu.run_publisher()

if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        pass
```

## ROS 2 Integration for Jetson

### Setting up ROS 2 Workspace

```bash
# Create workspace for Jetson-specific packages
mkdir -p ~/jetson_ws/src
cd ~/jetson_ws

# Install essential packages
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install ROS 2 Humble dependencies
sudo apt install ros-humble-ros-base
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-openni2-launch ros-humble-usb-cam

# Install Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-pointcloud-utils
sudo apt install ros-humble-isaac-ros-gxf
```

### Jetson Optimized Perception Pipeline

```python title="Optimized Perception Pipeline for Jetson"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import deque
import threading
import queue

class JetsonPerceptionNode(Node):
    def __init__(self):
        super().__init__('jetson_perception_node')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize queues for image processing
        self.image_queue = queue.Queue(maxsize=2)  # Limit queue size to prevent memory buildup
        self.processed_queue = queue.Queue(maxsize=2)
        
        # Subscribe to camera topics
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers for processed data
        self.detection_pub = self.create_publisher(Image, '/detections', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
        
        # Initialize processing thread
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()
        
        # FPS tracking
        self.fps_counter = deque(maxlen=30)
        self.frame_count = 0
        
        self.get_logger().info('Jetson Perception Node initialized')
    
    def image_callback(self, msg):
        """Callback for incoming images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Add to processing queue if there's space
            if not self.image_queue.full():
                self.image_queue.put_nowait(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def processing_loop(self):
        """Background thread for image processing"""
        while rclpy.ok():
            try:
                # Get image from queue
                if not self.image_queue.empty():
                    cv_image = self.image_queue.get_nowait()
                    
                    # Process image (example: simple color-based detection)
                    processed_image = self.process_frame(cv_image)
                    
                    # Publish result
                    if not self.processed_queue.full():
                        self.processed_queue.put_nowait(processed_image)
                    
                    # Update FPS
                    self.frame_count += 1
                    if len(self.fps_counter) < 30:
                        self.fps_counter.append(1.0)
                    else:
                        # Calculate FPS over last 30 frames
                        current_time = self.get_clock().now().nanoseconds / 1e9
                        self.fps_counter.append(current_time)
            
            except queue.Empty:
                # No image to process, sleep briefly
                time.sleep(0.001)
    
    def process_frame(self, frame):
        """Process a frame for perception tasks"""
        # Example: Color-based object detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define range for red color (adjust for your specific object)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_red = np.array([170, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        
        mask = mask1 + mask2
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours on original frame
        result_frame = frame.copy()
        cv2.drawContours(result_frame, contours, -1, (0, 255, 0), 2)
        
        # Draw center of largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(result_frame, (cx, cy), 7, (255, 255, 255), -1)
        
        return result_frame
    
    def publish_detection(self, processed_image):
        """Publish processed image"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_color_optical_frame"
            self.detection_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing detection: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JetsonPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down perception node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization on Jetson

### Power and Performance Management

```bash
# Check current power mode
sudo nvpmodel -q

# Set to maximum performance mode
sudo nvpmodel -m 0

# Lock clocks (if needed for consistent performance)
sudo jetson_clocks

# For thermal management
sudo tegrastats  # Monitor temperatures and power consumption

# Set CPU governor to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

### Memory and Resource Management

```python title="Resource Monitor for Jetson"
import psutil
import GPUtil
import time
import subprocess

def monitor_jetson_resources():
    """Monitor Jetson system resources"""
    print("Jetson System Resource Monitor")
    print("=" * 40)
    
    try:
        # Get CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        cpu_freq = psutil.cpu_freq()
        
        # Get memory usage
        memory = psutil.virtual_memory()
        
        # Get GPU usage (if available)
        gpus = GPUtil.getGPUs()
        gpu_load = gpus[0].load * 100 if gpus else 0
        gpu_mem = gpus[0].memoryUsed if gpus else 0
        gpu_temp = gpus[0].temperature if gpus else 0
        
        # Get Jetson-specific metrics
        jetson_metrics = get_jetson_metrics()
        
        print(f"CPU Usage: {cpu_percent}%")
        print(f"CPU Frequency: {cpu_freq.current:.0f} MHz")
        print(f"Memory: {memory.percent}% ({memory.used / 1024**3:.1f}GB / {memory.total / 1024**3:.1f}GB)")
        print(f"GPU Load: {gpu_load:.1f}%")
        print(f"GPU Memory: {gpu_mem} MB")
        print(f"GPU Temperature: {gpu_temp}°C")
        print(f"Jetson Power: {jetson_metrics.get('power', 'N/A')}W")
        print(f"Jetson Temp: {jetson_metrics.get('temp', 'N/A')}°C")
        
    except Exception as e:
        print(f"Error monitoring resources: {e}")

def get_jetson_metrics():
    """Get Jetson-specific metrics"""
    metrics = {}
    
    try:
        # Get power consumption (if available)
        # This is a simplified example - actual implementation depends on specific Jetson model
        result = subprocess.run(['cat', '/sys/bus/i2c/drivers/ina3221x/0-0040/iio:device0/in_power0_input'], 
                               capture_output=True, text=True)
        if result.returncode == 0:
            metrics['power'] = float(result.stdout.strip()) / 1000.0  # Convert to Watts
    except:
        metrics['power'] = 'N/A'
    
    try:
        # Get temperature
        result = subprocess.run(['cat', '/sys/class/thermal/thermal_zone0/temp'], 
                               capture_output=True, text=True)
        if result.returncode == 0:
            temp = int(result.stdout.strip()) / 1000.0
            metrics['temp'] = temp
    except:
        metrics['temp'] = 'N/A'
    
    return metrics

# Example usage
if __name__ == "__main__":
    while True:
        monitor_jetson_resources()
        time.sleep(5)
```

## Conclusion

The NVIDIA Jetson platform provides an excellent balance of computational power and power efficiency for Physical AI applications on robots. Proper setup and optimization ensure the platform can handle the demanding requirements of real-world robotic perception and control.

For humanoid robotics applications, the Jetson Orin series provides sufficient compute to run multiple neural networks simultaneously while maintaining real-time performance. The key is to match the specific Jetson model to your application requirements, considering factors like power consumption, thermal constraints, and computational needs.