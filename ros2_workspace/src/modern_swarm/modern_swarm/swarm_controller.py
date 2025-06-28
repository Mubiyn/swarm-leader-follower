#!/usr/bin/env python3

"""
Modern Swarm Controller ROS2 Node

This is the main controller node for the modern swarm leader-follower system.
It handles:
- Multi-robot formation control
- Leader motion planning  
- ROS2 topic communication
- Gazebo simulation integration
- RViz visualization support

Author: Modern Swarm Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from typing import Dict, List, Tuple

class Robot:
    """Robot state representation"""
    def __init__(self, name: str, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_update = time.time()

class SwarmController(Node):
    """Main ROS2 swarm controller node"""
    
    def __init__(self):
        super().__init__('swarm_controller')
        
        self.get_logger().info("🤖 Starting Modern Swarm Controller...")
        
        # Robot management
        self.robots: Dict[str, Robot] = {}
        self.robot_names = ['leader', 'follower_1', 'follower_2', 'follower_3']
        
        # Initialize robots
        for name in self.robot_names:
            self.robots[name] = Robot(name)
        
        # Formation parameters
        self.formations = {
            'triangle': [
                np.array([-2.5, -1.5]),  # follower_1
                np.array([-2.5, 1.5]),   # follower_2  
                np.array([-4.0, 0.0])    # follower_3
            ],
            'line': [
                np.array([-2.0, 0.0]),   # follower_1
                np.array([-4.0, 0.0]),   # follower_2
                np.array([-6.0, 0.0])    # follower_3
            ],
            'circle': [
                np.array([-3.0, 0.0]),   # follower_1
                np.array([1.5, 2.6]),    # follower_2
                np.array([1.5, -2.6])    # follower_3
            ]
        }
        self.current_formation = 'triangle'
        
        # Control parameters
        self.dt = 0.1
        self.leader_radius = 5.0
        self.leader_speed = 0.5
        self.kp_distance = 1.0
        self.kp_angle = 2.0
        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.5
        
        # ROS2 Publishers
        self.cmd_publishers = {}
        self.pose_publishers = {}
        
        for robot_name in self.robot_names:
            # Command velocity publishers
            self.cmd_publishers[robot_name] = self.create_publisher(
                Twist, f'/{robot_name}/cmd_vel', 10
            )
            
            # Pose publishers for visualization
            self.pose_publishers[robot_name] = self.create_publisher(
                PoseStamped, f'/{robot_name}/pose', 10
            )
        
        # ROS2 Subscribers
        self.odom_subscribers = {}
        for robot_name in self.robot_names:
            self.odom_subscribers[robot_name] = self.create_subscription(
                Odometry, f'/{robot_name}/odom', 
                lambda msg, name=robot_name: self.odom_callback(msg, name), 10
            )
        
        # Formation control publisher
        self.formation_pub = self.create_publisher(String, '/formation_command', 10)
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/swarm_status', 10)
        
        # Services for formation switching
        from std_srvs.srv import SetBool
        self.formation_service = self.create_service(
            SetBool, 'switch_formation', self.switch_formation_callback
        )
        
        # Main control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.start_time = time.time()
        
        self.get_logger().info("✅ Swarm Controller initialized successfully!")
        self.get_logger().info(f"📊 Managing {len(self.robot_names)} robots")
        self.get_logger().info(f"🎯 Current formation: {self.current_formation}")

    def odom_callback(self, msg: Odometry, robot_name: str):
        """Update robot state from odometry"""
        if robot_name in self.robots:
            robot = self.robots[robot_name]
            robot.x = msg.pose.pose.position.x
            robot.y = msg.pose.pose.position.y
            
            # Extract yaw from quaternion
            quat = msg.pose.pose.orientation
            robot.theta = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            robot.vx = msg.twist.twist.linear.x
            robot.vy = msg.twist.twist.linear.y
            robot.omega = msg.twist.twist.angular.z
            robot.last_update = time.time()

    def control_loop(self):
        """Main control loop"""
        current_time = time.time() - self.start_time
        
        # Update leader motion
        self.update_leader_motion(current_time)
        
        # Update followers
        self.update_followers()
        
        # Publish poses for visualization
        self.publish_poses()

    def update_leader_motion(self, t: float):
        """Update leader robot with circular motion"""
        leader = self.robots['leader']
        
        # Circular motion
        target_x = self.leader_radius * math.cos(self.leader_speed * t / self.leader_radius)
        target_y = self.leader_radius * math.sin(self.leader_speed * t / self.leader_radius)
        target_theta = self.leader_speed * t / self.leader_radius + math.pi/2
        
        # Simple position control for leader
        dx = target_x - leader.x
        dy = target_y - leader.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > 0.1:
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - leader.theta
            
            # Normalize angle
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            # Control commands
            linear_vel = min(self.kp_distance * distance, self.max_linear_vel)
            angular_vel = max(min(self.kp_angle * angle_error, self.max_angular_vel), 
                            -self.max_angular_vel)
        else:
            linear_vel = 0.0
            angular_vel = 0.0
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_publishers['leader'].publish(cmd)

    def update_followers(self):
        """Update follower robots with formation control"""
        leader = self.robots['leader']
        leader_pos = np.array([leader.x, leader.y])
        
        formation_offsets = self.formations[self.current_formation]
        follower_names = ['follower_1', 'follower_2', 'follower_3']
        
        for i, follower_name in enumerate(follower_names):
            if i < len(formation_offsets):
                follower = self.robots[follower_name]
                
                # Calculate target position
                offset = formation_offsets[i]
                target_pos = leader_pos + offset
                
                # Formation control
                dx = target_pos[0] - follower.x
                dy = target_pos[1] - follower.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance > 0.1:
                    angle_to_target = math.atan2(dy, dx)
                    angle_error = angle_to_target - follower.theta
                    
                    # Normalize angle
                    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                    
                    # Control commands
                    linear_vel = min(self.kp_distance * distance, self.max_linear_vel)
                    angular_vel = max(min(self.kp_angle * angle_error, self.max_angular_vel), 
                                    -self.max_angular_vel)
                    
                    # Inter-robot collision avoidance
                    avoidance_vel, avoidance_omega = self.collision_avoidance(follower, follower_name)
                    
                    linear_vel += avoidance_vel
                    angular_vel += avoidance_omega
                    
                    # Limit velocities
                    linear_vel = max(min(linear_vel, self.max_linear_vel), -self.max_linear_vel)
                    angular_vel = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
                else:
                    linear_vel = 0.0
                    angular_vel = 0.0
                
                # Publish command
                cmd = Twist()
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel
                self.cmd_publishers[follower_name].publish(cmd)

    def collision_avoidance(self, robot: Robot, robot_name: str) -> Tuple[float, float]:
        """Simple collision avoidance between robots"""
        avoidance_vel = 0.0
        avoidance_omega = 0.0
        safety_distance = 1.0
        
        for other_name, other_robot in self.robots.items():
            if other_name != robot_name:
                dx = robot.x - other_robot.x
                dy = robot.y - other_robot.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < safety_distance and distance > 0.01:
                    # Repulsion force
                    repulsion_strength = (safety_distance - distance) / safety_distance
                    avoidance_vel += (dx / distance) * repulsion_strength * 0.3
                    avoidance_omega += (dy / distance) * repulsion_strength * 0.2
        
        return avoidance_vel, avoidance_omega

    def publish_poses(self):
        """Publish robot poses for RViz visualization"""
        for robot_name, robot in self.robots.items():
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.position.x = robot.x
            pose_msg.pose.position.y = robot.y
            pose_msg.pose.position.z = 0.0
            
            # Convert theta to quaternion
            pose_msg.pose.orientation.w = math.cos(robot.theta / 2.0)
            pose_msg.pose.orientation.z = math.sin(robot.theta / 2.0)
            
            self.pose_publishers[robot_name].publish(pose_msg)

    def switch_formation_callback(self, request, response):
        """Service callback for formation switching"""
        formations = list(self.formations.keys())
        current_idx = formations.index(self.current_formation)
        next_idx = (current_idx + 1) % len(formations)
        self.current_formation = formations[next_idx]
        
        self.get_logger().info(f"🔄 Formation switched to: {self.current_formation}")
        
        response.success = True
        response.message = f"Formation switched to {self.current_formation}"
        return response

    def publish_status(self):
        """Publish system status"""
        status_msg = String()
        status_msg.data = f"Formation: {self.current_formation}, Robots: {len(self.robots)}"
        self.status_pub.publish(status_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        swarm_controller = SwarmController()
        rclpy.spin(swarm_controller)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down swarm controller...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 