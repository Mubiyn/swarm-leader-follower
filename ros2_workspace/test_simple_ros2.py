#!/usr/bin/env python3
"""
Simple ROS2 test script
Tests basic ROS2 functionality without complex dependencies
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SimpleTestNode(Node):
    """Simple test node for ROS2 functionality"""
    
    def __init__(self):
        super().__init__('simple_test_node')
        
        # Create a publisher
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        
        # Create a timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Create a subscriber
        self.subscription = self.create_subscription(
            String, '/test_topic', self.listener_callback, 10
        )
        
        self.counter = 0
        self.get_logger().info('Simple test node started')
    
    def timer_callback(self):
        """Timer callback to publish messages"""
        msg = String()
        msg.data = f'Hello ROS2! Message {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1
        
        if self.counter >= 5:
            self.get_logger().info('Test completed successfully!')
            rclpy.shutdown()
    
    def listener_callback(self, msg):
        """Subscriber callback to receive messages"""
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 