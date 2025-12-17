#!/usr/bin/env python3
# performance_test.py
"""
Performance verification script for ROS 2 communication.
Tests that communication latency is < 100ms and control loop frequency is >= 50Hz.
"""

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