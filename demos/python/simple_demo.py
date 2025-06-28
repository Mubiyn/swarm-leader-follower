#!/usr/bin/env python3
"""
SIMPLE DEMO - No ROS2 Dependencies
A minimal leader-follower system using just Python and matplotlib.
"""

import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque


class SimpleLeaderFollower:
    """Simple leader-follower simulation"""
    
    def __init__(self):
        # Leader position
        self.leader_x = 0.0
        self.leader_y = 0.0
        
        # Follower state
        self.follower_x = 0.0
        self.follower_y = 0.0
        self.follower_theta = 0.0
        
        # Control parameters
        self.desired_distance = 2.0
        self.k_linear = 0.3  # Reduced from 1.0 for smoother movement
        self.k_angular = 1.0  # Reduced from 2.0 for smoother turning
        
        # For visualization
        self.leader_trail_x = deque(maxlen=100)
        self.leader_trail_y = deque(maxlen=100)
        self.follower_trail_x = deque(maxlen=100)
        self.follower_trail_y = deque(maxlen=100)
        
        self.start_time = time.time()
        
    def update_leader(self):
        """Update leader position (circular motion)"""
        t = time.time() - self.start_time
        # Slower circular motion (0.3 instead of 0.5)
        self.leader_x = 3.0 * math.cos(0.3 * t)
        self.leader_y = 3.0 * math.sin(0.3 * t)
        
    def update_follower(self):
        """Update follower using simple control law"""
        dt = 0.1
        
        # Calculate distance and angle to leader
        dx = self.leader_x - self.follower_x
        dy = self.leader_y - self.follower_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        angle_to_leader = math.atan2(dy, dx)
        angle_error = angle_to_leader - self.follower_theta
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi
        
        # Calculate control commands
        if distance > self.desired_distance:
            linear_vel = self.k_linear * (distance - self.desired_distance)
        else:
            linear_vel = 0.0
            
        angular_vel = self.k_angular * angle_error
        
        # Limit velocities (reduced for smoother movement)
        linear_vel = max(-1.0, min(1.0, linear_vel))
        angular_vel = max(-1.5, min(1.5, angular_vel))
        
        # Update follower position
        self.follower_theta += angular_vel * dt
        self.follower_x += linear_vel * math.cos(self.follower_theta) * dt
        self.follower_y += linear_vel * math.sin(self.follower_theta) * dt
        
        # Store trails
        self.leader_trail_x.append(self.leader_x)
        self.leader_trail_y.append(self.leader_y)
        self.follower_trail_x.append(self.follower_x)
        self.follower_trail_y.append(self.follower_y)
        
        return linear_vel, angular_vel, distance
    
    def animate(self, frame):
        """Animation function"""
        # Update positions
        self.update_leader()
        linear_vel, angular_vel, distance = self.update_follower()
        
        # Clear plot
        plt.clf()
        
        # Set up plot
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.grid(True, alpha=0.3)
        plt.gca().set_aspect('equal')
        plt.title('🤖 Simple Leader-Follower Demo\n🔴 Leader   🔵 Follower', fontsize=14)
        
        # Plot trails
        if len(self.leader_trail_x) > 1:
            plt.plot(list(self.leader_trail_x), list(self.leader_trail_y), 
                    'r-', alpha=0.7, linewidth=2, label='Leader Path')
        
        if len(self.follower_trail_x) > 1:
            plt.plot(list(self.follower_trail_x), list(self.follower_trail_y), 
                    'b-', alpha=0.7, linewidth=2, label='Follower Path')
        
        # Plot robots
        plt.scatter(self.leader_x, self.leader_y, c='red', s=300, alpha=0.9, 
                   marker='o', label='Leader', edgecolors='darkred', linewidth=2)
        
        plt.scatter(self.follower_x, self.follower_y, c='blue', s=250, alpha=0.9, 
                   marker='o', label='Follower', edgecolors='darkblue', linewidth=2)
        
        # Draw follower orientation arrow
        arrow_length = 0.5
        arrow_x = arrow_length * math.cos(self.follower_theta)
        arrow_y = arrow_length * math.sin(self.follower_theta)
        plt.arrow(self.follower_x, self.follower_y, arrow_x, arrow_y,
                 head_width=0.2, head_length=0.1, fc='darkblue', ec='darkblue')
        
        # Add info boxes
        plt.text(-4.5, 4.5, f'Distance: {distance:.2f}m', fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        plt.text(-4.5, 4.0, f'Linear Vel: {linear_vel:.2f}m/s', fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
        
        plt.text(-4.5, 3.5, f'Angular Vel: {angular_vel:.2f}rad/s', fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
        
        # Add target distance circle
        target_circle = plt.Circle((self.leader_x, self.leader_y), self.desired_distance, 
                                 color='gray', fill=False, linestyle='--', alpha=0.5)
        plt.gca().add_patch(target_circle)
        
        plt.legend(loc='upper right')


def main():
    """Main function"""
    print("🚀 Starting Simple Leader-Follower Demo...")
    print("🔴 Red = Leader (moves in circle)")
    print("🔵 Blue = Follower (tries to stay 2m away)")
    print("Press Ctrl+C to stop")
    
    # Create simulation
    sim = SimpleLeaderFollower()
    
    # Create and run animation
    fig = plt.figure(figsize=(10, 10))
    ani = animation.FuncAnimation(fig, sim.animate, interval=100, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Stopping demo...")


if __name__ == '__main__':
    main() 