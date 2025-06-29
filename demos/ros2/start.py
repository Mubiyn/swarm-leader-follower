#!/usr/bin/env python3
"""
CLEAN START - Simple Leader-Follower System
A minimal, working implementation from scratch.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading


class LeaderRobot(Node):
    """Simple leader robot that moves in patterns"""
    
    def __init__(self):
        super().__init__('leader_robot')
        
        # Position
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
        """Move leader in a simple pattern"""
        t = time.time() - self.start_time
        
        # Slower circular motion (0.3 instead of 0.5)
        self.x = 3.0 * math.cos(0.3 * t)
        self.y = 3.0 * math.sin(0.3 * t)
        
        # Publish position
        pos_msg = Point()
        pos_msg.x = self.x
        pos_msg.y = self.y
        pos_msg.z = 0.0
        self.position_pub.publish(pos_msg)


class FollowerRobot(Node):
    """Simple follower robot that tracks the leader"""
    
    def __init__(self):
        super().__init__('follower_robot')
        
        # Position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Leader position
        self.leader_x = 0.0
        self.leader_y = 0.0
        
        # Subscribe to leader position
        self.leader_sub = self.create_subscription(
            Point, '/leader_position', self.leader_callback, 10)
        
        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, '/follower_cmd_vel', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Update position timer
        self.update_timer = self.create_timer(0.1, self.update_position)
        
        self.get_logger().info("🔵 Follower Robot Started!")
        
    def leader_callback(self, msg):
        """Update leader position"""
        self.leader_x = msg.x
        self.leader_y = msg.y
        
    def control_loop(self):
        """Simple proportional controller"""
        # Calculate distance to leader
        dx = self.leader_x - self.x
        dy = self.leader_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle to leader
        angle_to_leader = math.atan2(dy, dx)
        angle_error = angle_to_leader - self.theta
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi
        
        # Control gains (reduced for smoother movement)
        k_linear = 0.3
        k_angular = 1.0
        
        # Desired following distance
        desired_distance = 2.0
        
        # Calculate velocities
        cmd = Twist()
        
        # Linear velocity (move toward leader but stop at desired distance)
        if distance > desired_distance:
            cmd.linear.x = k_linear * (distance - desired_distance)
        else:
            cmd.linear.x = 0.0
            
        # Angular velocity (turn toward leader)
        cmd.angular.z = k_angular * angle_error
        
        # Limit velocities (reduced for smoother movement)
        cmd.linear.x = max(-1.0, min(1.0, cmd.linear.x))
        cmd.angular.z = max(-1.5, min(1.5, cmd.angular.z))
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
    def update_position(self):
        """Update follower position based on commands"""
        # This simulates robot movement (in real robots, this comes from sensors)
        dt = 0.1
        
        # Get the last command (simplified)
        # In a real system, you'd use odometry
        # For now, we'll use a simple kinematic model
        pass  # Position will be updated by the visualization


class SwarmVisualizer:
    """Real-time visualization of the swarm system"""
    
    def __init__(self, leader, follower):
        self.leader = leader
        self.follower = follower
        
        # Trails
        self.leader_trail_x = deque(maxlen=100)
        self.leader_trail_y = deque(maxlen=100)
        self.follower_trail_x = deque(maxlen=100)
        self.follower_trail_y = deque(maxlen=100)
        
        # Control variables for follower
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Subscribe to follower commands
        self.cmd_sub = self.follower.create_subscription(
            Twist, '/follower_cmd_vel', self.cmd_callback, 10)
        
    def cmd_callback(self, msg):
        """Update follower velocities"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
    def update_follower_position(self):
        """Update follower position based on control commands"""
        dt = 0.1
        
        # Update orientation
        self.follower.theta += self.angular_vel * dt
        
        # Update position
        self.follower.x += self.linear_vel * math.cos(self.follower.theta) * dt
        self.follower.y += self.linear_vel * math.sin(self.follower.theta) * dt
        
        # Store trails
        self.leader_trail_x.append(self.leader.x)
        self.leader_trail_y.append(self.leader.y)
        self.follower_trail_x.append(self.follower.x)
        self.follower_trail_y.append(self.follower.y)
        
    def animate(self, frame):
        """Animation function for matplotlib"""
        # Update follower position
        self.update_follower_position()
        
        # Clear plot
        plt.clf()
        
        # Set up plot
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.grid(True, alpha=0.3)
        plt.gca().set_aspect('equal')
        plt.title('🤖 Clean Start: Leader-Follower System\n🔴 Leader   🔵 Follower', fontsize=14)
        
        # Plot trails
        if len(self.leader_trail_x) > 1:
            plt.plot(list(self.leader_trail_x), list(self.leader_trail_y), 
                    'r-', alpha=0.7, linewidth=2, label='Leader Path')
        
        if len(self.follower_trail_x) > 1:
            plt.plot(list(self.follower_trail_x), list(self.follower_trail_y), 
                    'b-', alpha=0.7, linewidth=2, label='Follower Path')
        
        # Plot robots
        plt.scatter(self.leader.x, self.leader.y, c='red', s=300, alpha=0.9, 
                   marker='o', label='Leader', edgecolors='darkred', linewidth=2)
        
        plt.scatter(self.follower.x, self.follower.y, c='blue', s=250, alpha=0.9, 
                   marker='o', label='Follower', edgecolors='darkblue', linewidth=2)
        
        # Draw follower orientation arrow
        arrow_length = 0.5
        arrow_x = arrow_length * math.cos(self.follower.theta)
        arrow_y = arrow_length * math.sin(self.follower.theta)
        plt.arrow(self.follower.x, self.follower.y, arrow_x, arrow_y,
                 head_width=0.2, head_length=0.1, fc='darkblue', ec='darkblue')
        
        # Add distance info
        distance = math.sqrt((self.leader.x - self.follower.x)**2 + 
                           (self.leader.y - self.follower.y)**2)
        plt.text(-4.5, 4.5, f'Distance: {distance:.2f}m', fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        # Add velocity info
        plt.text(-4.5, 4.0, f'Linear Vel: {self.linear_vel:.2f}m/s', fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
        plt.text(-4.5, 3.5, f'Angular Vel: {self.angular_vel:.2f}rad/s', fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
        
        plt.legend(loc='upper right')


def run_ros_nodes():
    """Run ROS2 nodes in a separate thread"""
    rclpy.init()
    
    # Create nodes
    leader = LeaderRobot()
    follower = FollowerRobot()
    
    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(leader)
    executor.add_node(follower)
    
    # Store nodes globally so visualization can access them
    global global_leader, global_follower
    global_leader = leader
    global_follower = follower
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        leader.destroy_node()
        follower.destroy_node()
        rclpy.shutdown()


def main():
    """Main function"""
    print("🚀 Starting Clean Leader-Follower System...")
    print("🔴 Red circle = Leader (moves in circle)")
    print("🔵 Blue circle = Follower (tries to follow at 2m distance)")
    print("Press Ctrl+C to stop")
    
    # Start ROS2 nodes in a separate thread
    ros_thread = threading.Thread(target=run_ros_nodes, daemon=True)
    ros_thread.start()
    
    # Wait a moment for nodes to initialize
    time.sleep(1.0)
    
    # Create visualizer (needs access to global nodes)
    visualizer = SwarmVisualizer(global_leader, global_follower)
    
    # Create and run animation
    fig = plt.figure(figsize=(10, 10))
    ani = animation.FuncAnimation(fig, visualizer.animate, interval=100, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Stopping system...")
    

if __name__ == '__main__':
    main() 