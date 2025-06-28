#!/usr/bin/env python3
"""
🚧 Obstacle Avoidance ROS2 System
Complete ROS2 implementation with separate nodes for obstacle detection and avoidance
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

# ROS2 imports
from geometry_msgs.msg import Twist, Point, Pose2D, PointStamped
from std_msgs.msg import String, Bool, Int32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import rclpy.logging

@dataclass
class Robot:
    """Robot state representation"""
    x: float
    y: float
    theta: float
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    color: str = 'blue'
    marker: str = 'o'
    name: str = 'robot'

@dataclass 
class Obstacle:
    """Obstacle representation"""
    x: float
    y: float
    radius: float
    vx: float = 0.0  # For moving obstacles
    vy: float = 0.0
    color: str = 'red'
    is_dynamic: bool = False
    name: str = 'obstacle'

class LeaderRobotNode(Node):
    """ROS2 Leader Robot Node with Obstacle Avoidance"""
    
    def __init__(self):
        super().__init__('leader_robot')
        
        # Robot state
        self.robot = Robot(x=0, y=0, theta=0, color='red', marker='*', name='Leader')
        self.leader_speed = 0.3
        
        # Avoidance parameters
        self.obstacle_detection_range = 3.0
        self.obstacle_safety_distance = 1.5
        self.avoidance_strength = 2.0
        
        # Obstacle tracking
        self.obstacles = []
        
        # Publishers
        self.pose_publisher = self.create_publisher(Pose2D, '/leader_pose', 10)
        
        # Subscribers
        self.obstacles_sub = self.create_subscription(
            MarkerArray, '/obstacles', self.obstacles_callback, 10)
        
        # Timer for motion updates
        self.timer = self.create_timer(0.05, self.update_motion)
        
        self.get_logger().info('🔴 Leader Robot Node Started!')
        
    def obstacles_callback(self, msg):
        """Update obstacle information"""
        self.obstacles = []
        for marker in msg.markers:
            obstacle = Obstacle(
                x=marker.pose.position.x,
                y=marker.pose.position.y,
                radius=marker.scale.x / 2.0,  # Assuming circular obstacles
                is_dynamic=(marker.id >= 1000),  # Convention: dynamic obstacles have ID >= 1000
                name=marker.ns
            )
            self.obstacles.append(obstacle)
    
    def calculate_obstacle_avoidance(self) -> Tuple[float, float]:
        """Calculate avoidance forces for obstacles"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for obstacle in self.obstacles:
            # Calculate distance to obstacle
            dx = self.robot.x - obstacle.x
            dy = self.robot.y - obstacle.y
            distance = np.sqrt(dx**2 + dy**2)
            
            # Check if obstacle is within detection range
            if distance < self.obstacle_detection_range:
                # Calculate required safe distance
                required_distance = obstacle.radius + self.obstacle_safety_distance
                
                if distance < required_distance:
                    # Calculate repulsive force
                    if distance > 0.1:  # Avoid division by zero
                        force_magnitude = self.avoidance_strength * (required_distance - distance) / distance
                        avoidance_x += force_magnitude * dx
                        avoidance_y += force_magnitude * dy
                    else:
                        # Emergency push away
                        avoidance_x += self.avoidance_strength * np.random.uniform(-1, 1)
                        avoidance_y += self.avoidance_strength * np.random.uniform(-1, 1)
        
        # Limit avoidance force
        max_avoidance = 2.0
        avoidance_magnitude = np.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
            
        return avoidance_x, avoidance_y
        
    def update_motion(self):
        """Update leader motion with obstacle avoidance"""
        t = time.time()
        
        # Base circular motion
        radius = 3.0
        base_x = radius * np.cos(self.leader_speed * t)
        base_y = radius * np.sin(self.leader_speed * t)
        base_theta = self.leader_speed * t + np.pi/2
        
        # Apply obstacle avoidance to leader
        avoidance_x, avoidance_y = self.calculate_obstacle_avoidance()
        
        # Combine base motion with avoidance
        self.robot.x = base_x + avoidance_x
        self.robot.y = base_y + avoidance_y
        self.robot.theta = base_theta
        
        # Update leader velocity for feedback
        self.robot.vx = avoidance_x / 0.05 if 0.05 > 0 else 0
        self.robot.vy = avoidance_y / 0.05 if 0.05 > 0 else 0
        
        # Publish pose
        pose_msg = Pose2D()
        pose_msg.x = self.robot.x
        pose_msg.y = self.robot.y
        pose_msg.theta = self.robot.theta
        self.pose_publisher.publish(pose_msg)

class ObstacleManagerNode(Node):
    """ROS2 Node for managing static and dynamic obstacles"""
    
    def __init__(self):
        super().__init__('obstacle_manager')
        
        # Initialize obstacles
        self.static_obstacles = [
            Obstacle(x=3, y=2, radius=1.0, color='darkred', name='Static_1'),
            Obstacle(x=-6, y=-3, radius=1.5, color='darkred', name='Static_2'),
            Obstacle(x=1, y=-4, radius=0.8, color='darkred', name='Static_3'),
            Obstacle(x=-8, y=3, radius=1.2, color='darkred', name='Static_4')
        ]
        
        self.dynamic_obstacles = [
            Obstacle(x=-2, y=5, radius=0.8, vx=0.2, vy=-0.1, 
                    color='purple', is_dynamic=True, name='Dynamic_1'),
            Obstacle(x=6, y=-2, radius=1.0, vx=-0.15, vy=0.3,
                    color='purple', is_dynamic=True, name='Dynamic_2')
        ]
        
        # Publishers
        self.obstacles_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.collision_count_pub = self.create_publisher(Int32, '/collision_count', 10)
        
        # Collision tracking
        self.collision_count = 0
        
        # Robot tracking for collision detection
        self.robots = []
        
        # Subscribers for robot poses
        self.leader_pose_sub = self.create_subscription(
            Pose2D, '/leader_pose', self.leader_pose_callback, 10)
        self.follower_pose_subs = []
        for i in range(3):
            sub = self.create_subscription(
                Pose2D, f'/follower_{i+1}_pose', 
                lambda msg, idx=i: self.follower_pose_callback(msg, idx), 10)
            self.follower_pose_subs.append(sub)
        
        # Initialize robot list
        self.robots = [None] * 4  # Leader + 3 followers
        
        # Timer for obstacle updates
        self.timer = self.create_timer(0.05, self.update_obstacles)
        
        self.get_logger().info('🚧 Obstacle Manager Node Started!')
        
    def leader_pose_callback(self, msg):
        """Update leader pose"""
        if len(self.robots) > 0:
            self.robots[0] = Robot(x=msg.x, y=msg.y, theta=msg.theta, name='Leader')
        
    def follower_pose_callback(self, msg, follower_id):
        """Update follower pose"""
        if len(self.robots) > follower_id + 1:
            self.robots[follower_id + 1] = Robot(
                x=msg.x, y=msg.y, theta=msg.theta, name=f'Follower_{follower_id+1}')
    
    def update_dynamic_obstacles(self):
        """Update positions of dynamic obstacles"""
        dt = 0.05
        for obstacle in self.dynamic_obstacles:
            # Update position
            obstacle.x += obstacle.vx * dt
            obstacle.y += obstacle.vy * dt
            
            # Bounce off boundaries
            if obstacle.x > 8 or obstacle.x < -12:
                obstacle.vx *= -1
            if obstacle.y > 8 or obstacle.y < -8:
                obstacle.vy *= -1
    
    def check_collisions(self):
        """Check for collisions and update metrics"""
        valid_robots = [r for r in self.robots if r is not None]
        all_obstacles = self.static_obstacles + self.dynamic_obstacles
        
        # Check robot-obstacle collisions
        for robot in valid_robots:
            for obstacle in all_obstacles:
                dx = robot.x - obstacle.x
                dy = robot.y - obstacle.y
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance < obstacle.radius + 0.3:  # Robot radius ~0.3
                    self.collision_count += 1
        
        # Check robot-robot collisions
        for i, robot1 in enumerate(valid_robots):
            for robot2 in valid_robots[i+1:]:
                dx = robot1.x - robot2.x
                dy = robot1.y - robot2.y
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance < 0.6:  # Two robot radii
                    self.collision_count += 1
    
    def publish_obstacles(self):
        """Publish obstacle information as MarkerArray"""
        marker_array = MarkerArray()
        
        # Static obstacles
        for i, obstacle in enumerate(self.static_obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = obstacle.name
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obstacle.radius * 2
            marker.scale.y = obstacle.radius * 2
            marker.scale.z = 0.5
            
            marker.color.r = 0.8
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        # Dynamic obstacles
        for i, obstacle in enumerate(self.dynamic_obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = obstacle.name
            marker.id = 1000 + i  # Use ID >= 1000 for dynamic obstacles
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obstacle.radius * 2
            marker.scale.y = obstacle.radius * 2
            marker.scale.z = 0.5
            
            marker.color.r = 0.6
            marker.color.g = 0.0
            marker.color.b = 0.8
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        self.obstacles_pub.publish(marker_array)
        
        # Publish collision count
        collision_msg = Int32()
        collision_msg.data = self.collision_count
        self.collision_count_pub.publish(collision_msg)
    
    def update_obstacles(self):
        """Main update loop for obstacles"""
        # Update dynamic obstacles
        self.update_dynamic_obstacles()
        
        # Check collisions
        self.check_collisions()
        
        # Publish obstacle information
        self.publish_obstacles()

class FollowerRobotNode(Node):
    """ROS2 Follower Robot Node with Obstacle Avoidance"""
    
    def __init__(self, robot_id: int):
        super().__init__(f'follower_robot_{robot_id}')
        
        self.robot_id = robot_id
        self.robot = Robot(x=-2-robot_id, y=(-1)**robot_id * 2, theta=0, 
                          color=['blue', 'green', 'orange'][robot_id], 
                          marker=['o', 's', '^'][robot_id], 
                          name=f'Follower_{robot_id+1}')
        
        # Control parameters
        self.dt = 0.05
        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.5
        
        # Avoidance parameters
        self.obstacle_detection_range = 3.0
        self.robot_safety_distance = 1.0
        self.obstacle_safety_distance = 1.5
        self.avoidance_strength = 2.0
        
        # Formation parameters
        self.formations = {
            'triangle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'line': [(-2.0, 0.0), (-4.0, 0.0), (-6.0, 0.0)],
            'circle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'v_shape': [(-2.5, -2.0), (-2.5, 2.0), (-4.5, 0.0)]
        }
        self.current_formation = 'triangle'
        
        # State tracking
        self.leader_pose = None
        self.obstacles = []
        self.other_robots = []
        
        # Subscribers
        self.leader_pose_sub = self.create_subscription(
            Pose2D, '/leader_pose', self.leader_pose_callback, 10)
        self.obstacles_sub = self.create_subscription(
            MarkerArray, '/obstacles', self.obstacles_callback, 10)
        self.formation_sub = self.create_subscription(
            String, '/formation_command', self.formation_callback, 10)
        
        # Subscribe to other robot poses for collision avoidance
        for i in range(3):
            if i != robot_id:
                sub = self.create_subscription(
                    Pose2D, f'/follower_{i+1}_pose',
                    lambda msg, idx=i: self.other_robot_pose_callback(msg, idx), 10)
        
        # Publishers
        self.pose_publisher = self.create_publisher(Pose2D, f'/follower_{robot_id+1}_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/follower_{robot_id+1}/cmd_vel', 10)
        
        # Timer for control updates
        self.timer = self.create_timer(self.dt, self.update_control)
        
        self.get_logger().info(f'🔵 Follower Robot {robot_id+1} Started!')
        
    def leader_pose_callback(self, msg):
        """Update leader pose"""
        self.leader_pose = msg
        
    def obstacles_callback(self, msg):
        """Update obstacle information"""
        self.obstacles = []
        for marker in msg.markers:
            obstacle = Obstacle(
                x=marker.pose.position.x,
                y=marker.pose.position.y,
                radius=marker.scale.x / 2.0,
                is_dynamic=(marker.id >= 1000),
                name=marker.ns
            )
            self.obstacles.append(obstacle)
            
    def other_robot_pose_callback(self, msg, robot_id):
        """Update other robot poses for collision avoidance"""
        robot = Robot(x=msg.x, y=msg.y, theta=msg.theta, name=f'Robot_{robot_id}')
        
        # Update or add robot
        found = False
        for i, other_robot in enumerate(self.other_robots):
            if other_robot.name == robot.name:
                self.other_robots[i] = robot
                found = True
                break
        if not found:
            self.other_robots.append(robot)
        
    def formation_callback(self, msg):
        """Update formation command"""
        if msg.data in self.formations:
            self.current_formation = msg.data
            self.get_logger().info(f'🔄 Switched to {msg.data.upper()} formation')
    
    def get_formation_target(self) -> Tuple[float, float]:
        """Get target position for this follower in current formation"""
        if not self.leader_pose:
            return self.robot.x, self.robot.y
            
        leader_x, leader_y = self.leader_pose.x, self.leader_pose.y
        leader_theta = self.leader_pose.theta
        
        # Get formation offset
        formation_offsets = self.formations[self.current_formation]
        if self.robot_id >= len(formation_offsets):
            return leader_x - 2.0, leader_y
            
        dx, dy = formation_offsets[self.robot_id]
        
        # Rotate offset based on leader orientation
        cos_theta = np.cos(leader_theta)
        sin_theta = np.sin(leader_theta)
        
        target_x = leader_x + dx * cos_theta - dy * sin_theta
        target_y = leader_y + dx * sin_theta + dy * cos_theta
        
        return target_x, target_y
    
    def calculate_obstacle_avoidance(self) -> Tuple[float, float]:
        """Calculate avoidance forces for obstacles"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for obstacle in self.obstacles:
            # Calculate distance to obstacle
            dx = self.robot.x - obstacle.x
            dy = self.robot.y - obstacle.y
            distance = np.sqrt(dx**2 + dy**2)
            
            # Check if obstacle is within detection range
            if distance < self.obstacle_detection_range:
                # Calculate required safe distance
                required_distance = obstacle.radius + self.obstacle_safety_distance
                
                if distance < required_distance:
                    # Calculate repulsive force
                    if distance > 0.1:  # Avoid division by zero
                        force_magnitude = self.avoidance_strength * (required_distance - distance) / distance
                        avoidance_x += force_magnitude * dx
                        avoidance_y += force_magnitude * dy
                    else:
                        # Emergency push away
                        avoidance_x += self.avoidance_strength * np.random.uniform(-1, 1)
                        avoidance_y += self.avoidance_strength * np.random.uniform(-1, 1)
        
        # Limit avoidance force
        max_avoidance = 2.0
        avoidance_magnitude = np.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
            
        return avoidance_x, avoidance_y
    
    def calculate_robot_avoidance(self) -> Tuple[float, float]:
        """Calculate avoidance forces for other robots"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        # Include leader in robot avoidance
        all_other_robots = self.other_robots.copy()
        if self.leader_pose:
            leader_robot = Robot(x=self.leader_pose.x, y=self.leader_pose.y, 
                               theta=self.leader_pose.theta, name='Leader')
            all_other_robots.append(leader_robot)
        
        for other in all_other_robots:
            dx = self.robot.x - other.x
            dy = self.robot.y - other.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < self.robot_safety_distance and distance > 0.1:
                force_magnitude = 0.5 * (self.robot_safety_distance - distance) / distance
                avoidance_x += force_magnitude * dx
                avoidance_y += force_magnitude * dy
        
        return avoidance_x, avoidance_y
    
    def update_control(self):
        """Update follower control with obstacle avoidance"""
        target_x, target_y = self.get_formation_target()
        
        # Calculate formation attraction
        error_x = target_x - self.robot.x
        error_y = target_y - self.robot.y  
        distance_error = np.sqrt(error_x**2 + error_y**2)
        
        # Formation control gains
        kp_formation = 0.4
        
        # Calculate formation forces
        if distance_error > 0.01:
            formation_x = kp_formation * error_x
            formation_y = kp_formation * error_y
        else:
            formation_x = formation_y = 0.0
        
        # Calculate obstacle avoidance forces
        avoidance_x, avoidance_y = self.calculate_obstacle_avoidance()
        
        # Calculate robot-robot avoidance forces
        robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance()
        
        # Combine all forces
        total_vx = formation_x + avoidance_x + robot_avoidance_x
        total_vy = formation_y + avoidance_y + robot_avoidance_y
        
        # Limit velocities
        total_vx = np.clip(total_vx, -self.max_linear_vel, self.max_linear_vel)
        total_vy = np.clip(total_vy, -self.max_linear_vel, self.max_linear_vel)
        
        # Update robot state
        self.robot.vx = total_vx
        self.robot.vy = total_vy
        
        # Update position
        self.robot.x += self.robot.vx * self.dt
        self.robot.y += self.robot.vy * self.dt
        
        # Update orientation to face movement direction
        if abs(self.robot.vx) > 0.1 or abs(self.robot.vy) > 0.1:
            self.robot.theta = np.arctan2(self.robot.vy, self.robot.vx)
        
        # Publish pose
        pose_msg = Pose2D()
        pose_msg.x = self.robot.x
        pose_msg.y = self.robot.y
        pose_msg.theta = self.robot.theta
        self.pose_publisher.publish(pose_msg)
        
        # Publish cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(np.sqrt(self.robot.vx**2 + self.robot.vy**2))
        cmd_vel_msg.angular.z = float(self.robot.omega)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

class ControllerNode(Node):
    """ROS2 Main Controller Node"""
    
    def __init__(self):
        super().__init__('controller_node')
        
        # Publishers
        self.formation_pub = self.create_publisher(String, '/formation_command', 10)
        
        # Formation control
        self.formations = ['triangle', 'line', 'circle', 'v_shape']
        self.formation_index = 0
        
        # Collision monitoring
        self.collision_count = 0
        
        # Subscribers
        self.collision_sub = self.create_subscription(
            Int32, '/collision_count', self.collision_callback, 10)
        
        self.get_logger().info('🎮 Controller Node Started!')
        
        # Timer for formation cycling
        self.timer = self.create_timer(8.0, self.cycle_formation)
        
    def collision_callback(self, msg):
        """Update collision count"""
        self.collision_count = msg.data
        
    def cycle_formation(self):
        """Automatically cycle through formations for demo"""
        formation = self.formations[self.formation_index]
        
        msg = String()
        msg.data = formation
        self.formation_pub.publish(msg)
        
        self.get_logger().info(f'🔄 Switching to {formation.upper()} formation | Collisions: {self.collision_count}')
        
        self.formation_index = (self.formation_index + 1) % len(self.formations)

def main():
    """Main function to run all ROS2 nodes"""
    rclpy.init()
    
    try:
        # Create nodes
        leader_node = LeaderRobotNode()
        obstacle_manager = ObstacleManagerNode()
        controller_node = ControllerNode()
        
        # Create follower nodes
        follower_nodes = []
        for i in range(3):
            follower_nodes.append(FollowerRobotNode(i))
        
        # Create executor and add nodes
        executor = MultiThreadedExecutor()
        executor.add_node(leader_node)
        executor.add_node(obstacle_manager)
        executor.add_node(controller_node)
        for follower in follower_nodes:
            executor.add_node(follower)
        
        print("🚀 Starting Obstacle Avoidance ROS2 Multi-Robot System...")
        print("🔴 Leader with obstacle avoidance on /leader_pose")
        print("🚧 Obstacle manager publishing to /obstacles")
        print("🔵🟢🟠 Followers with formation + obstacle avoidance")
        print("🎮 Controller cycling formations every 8 seconds")
        print("📡 Topics: /leader_pose, /obstacles, /formation_command, /collision_count")
        print("🚧 Static red obstacles + moving purple obstacles")
        print("Press Ctrl+C to stop")
        
        # Spin executor
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping ROS2 nodes...")
    finally:
        # Cleanup
        leader_node.destroy_node()
        obstacle_manager.destroy_node()
        controller_node.destroy_node()
        for follower in follower_nodes:
            follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 