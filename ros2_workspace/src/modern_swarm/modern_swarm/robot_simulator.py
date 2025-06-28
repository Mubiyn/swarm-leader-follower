#!/usr/bin/env python3

"""
Robot Simulator Node

This node simulates a single robot for when not using Gazebo.
It publishes odometry and listens to cmd_vel commands.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time

class RobotSimulator(Node):
    """Simulates a robot's motion and publishes odometry"""
    
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Get parameters
        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.theta = self.get_parameter('initial_theta').get_parameter_value().double_value
        
        # Robot state
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Subscribers  
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10
        )
        
        # Timer for simulation updates
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.update_simulation)
        
        self.get_logger().info(f"🤖 Robot simulator started: {self.robot_name}")
        
    def cmd_callback(self, msg):
        """Handle velocity commands"""
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z
        
    def update_simulation(self):
        """Update robot position and publish odometry"""
        # Update position (simple bicycle model)
        self.x += self.vx * math.cos(self.theta) * self.dt
        self.y += self.vx * math.sin(self.theta) * self.dt
        self.theta += self.vtheta * self.dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'{self.robot_name}/base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vtheta
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    simulator = RobotSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 