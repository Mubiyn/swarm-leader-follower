#!/usr/bin/env python3
"""
MULTI-FOLLOWER ROS2 - Multiple Robots Following Leader (ROS2 Version)
ROS2 implementation with 1 leader + 3 followers in formation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String
import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading


class LeaderRobot(Node):
    """Leader robot that publishes its position"""
    
    def __init__(self):
        super().__init__('leader_robot')
        
        # Position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Publisher for leader position
        self.position_pub = self.create_publisher(Point, '/leader_position', 10)
        
        # Timer to update position
        self.timer = self.create_timer(0.1, self.update_position)
        self.start_time = time.time()
        
        self.get_logger().info("🔴 Leader Robot Started!")
        
    def update_position(self):
        """Move leader in circular pattern"""
        t = time.time() - self.start_time
        
        # Slower circular motion (0.3 instead of 0.5)
        self.x = 3.0 * math.cos(0.3 * t)
        self.y = 3.0 * math.sin(0.3 * t)
        self.theta = 0.3 * t + math.pi/2
        
        # Publish position
        pos_msg = Point()
        pos_msg.x = self.x
        pos_msg.y = self.y
        pos_msg.z = self.theta  # Using z for heading
        self.position_pub.publish(pos_msg)


class FollowerRobot(Node):
    """Follower robot that maintains formation position"""
    
    def __init__(self, robot_id, formation_offset):
        super().__init__(f'follower_robot_{robot_id}')
        
        self.robot_id = robot_id
        self.formation_offset = formation_offset
        
        # Position and orientation
        self.x = formation_offset["x_offset"]
        self.y = formation_offset["y_offset"]
        self.theta = 0.0
        
        # Leader information
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_theta = 0.0
        
        # Other robots' positions (for collision avoidance)
        self.other_robots = {}
        
        # Subscribe to leader position
        self.leader_sub = self.create_subscription(
            Point, '/leader_position', self.leader_callback, 10)
        
        # Subscribe to other followers' positions
        self.others_sub = self.create_subscription(
            PoseStamped, '/robot_positions', self.others_callback, 10)
        
        # Publisher for robot commands and position
        self.cmd_pub = self.create_publisher(Twist, f'/robot_{robot_id}/cmd_vel', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/robot_positions', 10)
        
        # Control parameters (reduced for smoother movement)
        self.k_linear = 0.3
        self.k_angular = 1.0
        self.collision_radius = 1.0
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.position_timer = self.create_timer(0.1, self.publish_position)
        
        self.get_logger().info(f"🔵 Follower Robot {robot_id} Started!")
        
    def leader_callback(self, msg):
        """Update leader position"""
        self.leader_x = msg.x
        self.leader_y = msg.y
        self.leader_theta = msg.z
        
    def others_callback(self, msg):
        """Update other robots' positions"""
        robot_id = msg.header.frame_id
        if robot_id != f"robot_{self.robot_id}":
            self.other_robots[robot_id] = {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y
            }
    
    def publish_position(self):
        """Publish own position for others to see"""
        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = f"robot_{self.robot_id}"
        pos_msg.pose.position.x = self.x
        pos_msg.pose.position.y = self.y
        pos_msg.pose.position.z = 0.0
        self.pos_pub.publish(pos_msg)
        
    def calculate_formation_target(self):
        """Calculate target position in formation"""
        # Rotate formation offset by leader's heading
        cos_h = math.cos(self.leader_theta)
        sin_h = math.sin(self.leader_theta)
        
        rotated_x = (self.formation_offset["x_offset"] * cos_h - 
                    self.formation_offset["y_offset"] * sin_h)
        rotated_y = (self.formation_offset["x_offset"] * sin_h + 
                    self.formation_offset["y_offset"] * cos_h)
        
        target_x = self.leader_x + rotated_x
        target_y = self.leader_y + rotated_y
        
        return target_x, target_y
    
    def calculate_collision_avoidance(self):
        """Calculate collision avoidance forces"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        # Check against leader
        dx = self.leader_x - self.x
        dy = self.leader_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.collision_radius and distance > 0:
            force_magnitude = (self.collision_radius - distance) / self.collision_radius
            avoidance_x -= force_magnitude * (dx / distance)
            avoidance_y -= force_magnitude * (dy / distance)
        
        # Check against other followers
        for robot_id, pos in self.other_robots.items():
            dx = pos['x'] - self.x
            dy = pos['y'] - self.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < self.collision_radius and distance > 0:
                force_magnitude = (self.collision_radius - distance) / self.collision_radius
                avoidance_x -= force_magnitude * (dx / distance)
                avoidance_y -= force_magnitude * (dy / distance)
        
        return avoidance_x, avoidance_y
        
    def control_loop(self):
        """Main control loop"""
        dt = 0.1
        
        # Get target position
        target_x, target_y = self.calculate_formation_target()
        
        # Calculate distance and angle to target
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.theta
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi
        
        # Basic control
        linear_vel = self.k_linear * distance
        angular_vel = self.k_angular * angle_error
        
        # Add collision avoidance
        avoidance_x, avoidance_y = self.calculate_collision_avoidance()
        avoidance_strength = 1.0  # Reduced from 2.0 for smoother avoidance
        linear_vel += avoidance_strength * avoidance_x
        angular_vel += avoidance_strength * avoidance_y
        
        # Limit velocities (reduced for smoother movement)
        linear_vel = max(-1.0, min(1.0, linear_vel))
        angular_vel = max(-1.5, min(1.5, angular_vel))
        
        # Update position (in real robots, this comes from odometry)
        self.theta += angular_vel * dt
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)


class MultiRobotVisualizer:
    """Visualizer for multi-robot system"""
    
    def __init__(self, leader, followers):
        self.leader = leader
        self.followers = followers
        
        # Trails
        self.leader_trail_x = deque(maxlen=100)
        self.leader_trail_y = deque(maxlen=100)
        self.follower_trails = []
        
        for _ in followers:
            self.follower_trails.append({
                'x': deque(maxlen=100),
                'y': deque(maxlen=100)
            })
        
    def animate(self, frame):
        """Animation function"""
        # Store trails
        self.leader_trail_x.append(self.leader.x)
        self.leader_trail_y.append(self.leader.y)
        
        for i, follower in enumerate(self.followers):
            self.follower_trails[i]['x'].append(follower.x)
            self.follower_trails[i]['y'].append(follower.y)
        
        # Clear plot
        plt.clf()
        
        # Set up plot
        plt.xlim(-6, 6)
        plt.ylim(-6, 6)
        plt.grid(True, alpha=0.3)
        plt.gca().set_aspect('equal')
        plt.title('🤖 Multi-Robot ROS2 Formation\n🔴 Leader  🔵🟢🟠 3 Followers', fontsize=14)
        
        # Plot leader
        plt.scatter(self.leader.x, self.leader.y, c='red', s=400, alpha=0.9, 
                   marker='*', label='Leader', edgecolors='darkred', linewidth=3)
        
        # Plot leader trail
        if len(self.leader_trail_x) > 1:
            plt.plot(list(self.leader_trail_x), list(self.leader_trail_y), 
                    'r-', alpha=0.7, linewidth=2)
        
        # Plot followers
        colors = ['blue', 'green', 'orange']
        markers = ['o', 's', '^']
        names = ['Follower 1', 'Follower 2', 'Follower 3']
        
        for i, follower in enumerate(self.followers):
            # Plot robot
            plt.scatter(follower.x, follower.y, c=colors[i], s=300, alpha=0.9, 
                       marker=markers[i], label=names[i], 
                       edgecolors='black', linewidth=2)
            
            # Plot trail
            trail = self.follower_trails[i]
            if len(trail['x']) > 1:
                plt.plot(list(trail['x']), list(trail['y']), 
                        color=colors[i], alpha=0.7, linewidth=2)
            
            # Plot orientation arrow
            arrow_length = 0.4
            arrow_x = arrow_length * math.cos(follower.theta)
            arrow_y = arrow_length * math.sin(follower.theta)
            plt.arrow(follower.x, follower.y, arrow_x, arrow_y,
                     head_width=0.15, head_length=0.1, 
                     fc=colors[i], ec=colors[i], alpha=0.8)
            
            # Plot target position
            target_x, target_y = follower.calculate_formation_target()
            plt.scatter(target_x, target_y, c=colors[i], s=100, alpha=0.3, 
                       marker='x', linewidth=3)
        
        # Add formation lines
        for follower in self.followers:
            target_x, target_y = follower.calculate_formation_target()
            plt.plot([self.leader.x, target_x], [self.leader.y, target_y], 
                    'k--', alpha=0.3, linewidth=1)
        
        # Add info
        info_text = f"ROS2 Robots: {len(self.followers)+1}\n"
        for i, follower in enumerate(self.followers):
            target_x, target_y = follower.calculate_formation_target()
            distance = math.sqrt((target_x - follower.x)**2 + (target_y - follower.y)**2)
            info_text += f"F{i+1}: {distance:.1f}m to target\n"
        
        plt.text(-5.5, 5.5, info_text, fontsize=9, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8),
                verticalalignment='top')
        
        plt.legend(loc='upper right')


def run_ros_nodes():
    """Run all ROS2 nodes"""
    rclpy.init()
    
    # Create leader
    leader = LeaderRobot()
    
    # Create followers with formation offsets
    formation_offsets = [
        {"x_offset": -1.5, "y_offset": -2.0},  # Left rear
        {"x_offset": 1.5, "y_offset": -2.0},   # Right rear
        {"x_offset": 0.0, "y_offset": -3.0}    # Center rear
    ]
    
    followers = []
    for i, offset in enumerate(formation_offsets):
        follower = FollowerRobot(i+1, offset)
        followers.append(follower)
    
    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(leader)
    for follower in followers:
        executor.add_node(follower)
    
    # Store globally for visualization
    global global_leader, global_followers
    global_leader = leader
    global_followers = followers
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        leader.destroy_node()
        for follower in followers:
            follower.destroy_node()
        rclpy.shutdown()


def main():
    """Main function"""
    print("🚀 Starting Multi-Robot ROS2 System...")
    print("🔴 Red star = Leader")
    print("🔵 Blue circle = Follower 1 (left rear)")
    print("🟢 Green square = Follower 2 (right rear)")
    print("🟠 Orange triangle = Follower 3 (center rear)")
    print("❌ X marks = Target formation positions")
    print("Press Ctrl+C to stop")
    
    # Start ROS2 nodes in separate thread
    ros_thread = threading.Thread(target=run_ros_nodes, daemon=True)
    ros_thread.start()
    
    # Wait for nodes to initialize
    time.sleep(1.0)
    
    # Create visualizer
    visualizer = MultiRobotVisualizer(global_leader, global_followers)
    
    # Create and run animation
    fig = plt.figure(figsize=(12, 10))
    ani = animation.FuncAnimation(fig, visualizer.animate, interval=100, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Stopping ROS2 multi-robot system...")


if __name__ == '__main__':
    main() 