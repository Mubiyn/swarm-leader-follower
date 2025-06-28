#!/usr/bin/env python3
"""
MULTI-FOLLOWER DEMO - Multiple Robots Following Leader
Extends simple_demo.py to support 1 leader + 3 followers in formation.
"""

import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque


class Robot:
    """Generic robot class for both leader and followers"""
    
    def __init__(self, robot_id, color, initial_x=0.0, initial_y=0.0):
        self.id = robot_id
        self.color = color
        self.x = initial_x
        self.y = initial_y
        self.theta = 0.0
        
        # Trail for visualization
        self.trail_x = deque(maxlen=100)
        self.trail_y = deque(maxlen=100)


class MultiRobotSystem:
    """Multi-robot leader-follower system"""
    
    def __init__(self):
        # Create robots
        self.leader = Robot("leader", "red", 0.0, 0.0)
        
        # Create 3 followers with different starting positions
        self.followers = [
            Robot("follower_1", "blue", -2.0, -2.0),    # Rear left
            Robot("follower_2", "green", 2.0, -2.0),    # Rear right  
            Robot("follower_3", "orange", 0.0, -4.0)    # Rear center
        ]
        
        # Formation configurations
        self.formations = {
            'triangle': [
                {"x_offset": -1.5, "y_offset": -2.0},  # Left rear
                {"x_offset": 1.5, "y_offset": -2.0},   # Right rear
                {"x_offset": 0.0, "y_offset": -3.0}    # Center rear
            ],
            'line': [
                {"x_offset": 0.0, "y_offset": -2.0},   # Rear center
                {"x_offset": 0.0, "y_offset": -4.0},   # Further rear
                {"x_offset": 0.0, "y_offset": -6.0}    # Furthest rear
            ],
            'circle': [
                {"x_offset": -2.0, "y_offset": 0.0},   # Left
                {"x_offset": 2.0, "y_offset": 0.0},    # Right
                {"x_offset": 0.0, "y_offset": -2.0}    # Rear
            ],
            'v_shape': [
                {"x_offset": -2.0, "y_offset": -2.0},  # Left rear
                {"x_offset": 2.0, "y_offset": -2.0},   # Right rear
                {"x_offset": 0.0, "y_offset": -4.0}    # Center rear
            ]
        }
        
        self.current_formation = 'triangle'
        self.formation_positions = self.formations[self.current_formation]
        
        # Control parameters (reduced for smoother movement)
        self.k_linear = 0.3
        self.k_angular = 1.0
        self.k_formation = 0.5  # Formation keeping gain
        
        self.start_time = time.time()
        
        # Formation switching
        self.formation_names = list(self.formations.keys())
        self.formation_index = 0
        
    def update_leader(self):
        """Update leader position (circular motion)"""
        t = time.time() - self.start_time
        # Slower circular motion (0.3 instead of 0.5)
        self.leader.x = 3.0 * math.cos(0.3 * t)
        self.leader.y = 3.0 * math.sin(0.3 * t)
        
        # Store trail
        self.leader.trail_x.append(self.leader.x)
        self.leader.trail_y.append(self.leader.y)
        
    def switch_formation(self):
        """Switch to next formation pattern"""
        self.formation_index = (self.formation_index + 1) % len(self.formation_names)
        self.current_formation = self.formation_names[self.formation_index]
        self.formation_positions = self.formations[self.current_formation]
        
    def calculate_formation_target(self, follower_idx):
        """Calculate target position for a follower in formation"""
        formation = self.formation_positions[follower_idx]
        
        # Calculate leader's heading (for formation orientation)
        t = time.time() - self.start_time
        leader_heading = 0.3 * t + math.pi/2  # Perpendicular to velocity (slower)
        
        # Rotate formation offset by leader's heading
        cos_h = math.cos(leader_heading)
        sin_h = math.sin(leader_heading)
        
        rotated_x = formation["x_offset"] * cos_h - formation["y_offset"] * sin_h
        rotated_y = formation["x_offset"] * sin_h + formation["y_offset"] * cos_h
        
        target_x = self.leader.x + rotated_x
        target_y = self.leader.y + rotated_y
        
        return target_x, target_y
        
    def update_follower(self, follower, follower_idx):
        """Update individual follower using formation control"""
        dt = 0.1
        
        # Get target position in formation
        target_x, target_y = self.calculate_formation_target(follower_idx)
        
        # Calculate distance and angle to target
        dx = target_x - follower.x
        dy = target_y - follower.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - follower.theta
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi
        
        # Control law
        linear_vel = self.k_linear * distance
        angular_vel = self.k_angular * angle_error
        
        # Add inter-robot collision avoidance
        avoidance_x, avoidance_y = self.calculate_collision_avoidance(follower)
        
        # Modify control commands for collision avoidance
        avoidance_strength = 1.0  # Reduced from 2.0 for smoother avoidance
        linear_vel += avoidance_strength * avoidance_x
        angular_vel += avoidance_strength * avoidance_y
        
        # Limit velocities (reduced for smoother movement)
        linear_vel = max(-1.0, min(1.0, linear_vel))
        angular_vel = max(-1.5, min(1.5, angular_vel))
        
        # Update follower position
        follower.theta += angular_vel * dt
        follower.x += linear_vel * math.cos(follower.theta) * dt
        follower.y += linear_vel * math.sin(follower.theta) * dt
        
        # Store trail
        follower.trail_x.append(follower.x)
        follower.trail_y.append(follower.y)
        
        return linear_vel, angular_vel, distance
    
    def calculate_collision_avoidance(self, current_robot):
        """Calculate collision avoidance forces"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        collision_radius = 1.0  # Minimum distance between robots
        
        # Check against all other robots
        all_robots = [self.leader] + self.followers
        
        for other_robot in all_robots:
            if other_robot.id == current_robot.id:
                continue
                
            dx = other_robot.x - current_robot.x
            dy = other_robot.y - current_robot.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < collision_radius and distance > 0:
                # Repulsive force (move away from other robot)
                force_magnitude = (collision_radius - distance) / collision_radius
                avoidance_x -= force_magnitude * (dx / distance)
                avoidance_y -= force_magnitude * (dy / distance)
        
        return avoidance_x, avoidance_y
    
    def update_all_robots(self):
        """Update all robots in the system"""
        self.update_leader()
        
        follower_data = []
        for i, follower in enumerate(self.followers):
            vel_data = self.update_follower(follower, i)
            follower_data.append(vel_data)
            
        return follower_data
    
    def animate(self, frame):
        """Animation function for matplotlib"""
        follower_data = self.update_all_robots()
        
        # Clear plot
        plt.clf()
        
        # Set up plot
        plt.xlim(-6, 6)
        plt.ylim(-6, 6)
        plt.grid(True, alpha=0.3)
        plt.gca().set_aspect('equal')
        plt.title(f'🤖 Multi-Robot Formation Demo\n🔴 Leader  🔵🟢🟠 3 Followers\nFormation: {self.current_formation.upper()} (Press SPACE to switch)', fontsize=14)
        
        # Plot leader
        plt.scatter(self.leader.x, self.leader.y, c='red', s=400, alpha=0.9, 
                   marker='*', label='Leader', edgecolors='darkred', linewidth=3)
        
        # Plot leader trail
        if len(self.leader.trail_x) > 1:
            plt.plot(list(self.leader.trail_x), list(self.leader.trail_y), 
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
            if len(follower.trail_x) > 1:
                plt.plot(list(follower.trail_x), list(follower.trail_y), 
                        color=colors[i], alpha=0.7, linewidth=2)
            
            # Plot orientation arrow
            arrow_length = 0.4
            arrow_x = arrow_length * math.cos(follower.theta)
            arrow_y = arrow_length * math.sin(follower.theta)
            plt.arrow(follower.x, follower.y, arrow_x, arrow_y,
                     head_width=0.15, head_length=0.1, 
                     fc=colors[i], ec=colors[i], alpha=0.8)
            
            # Plot target position
            target_x, target_y = self.calculate_formation_target(i)
            plt.scatter(target_x, target_y, c=colors[i], s=100, alpha=0.3, 
                       marker='x', linewidth=3)
        
        # Add formation lines (connecting targets to leader)
        for i in range(len(self.followers)):
            target_x, target_y = self.calculate_formation_target(i)
            plt.plot([self.leader.x, target_x], [self.leader.y, target_y], 
                    'k--', alpha=0.3, linewidth=1)
        
        # Add info box
        info_text = f"Formation: {self.current_formation.upper()}\n"
        info_text += f"Robots: {len(self.followers)+1}\n"
        for i, (linear_vel, angular_vel, distance) in enumerate(follower_data):
            info_text += f"F{i+1}: dist={distance:.1f}m, v={linear_vel:.1f}\n"
        info_text += "\nControls:\nSPACE: Switch formation"
        
        plt.text(-5.5, 5.5, info_text, fontsize=9, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8),
                verticalalignment='top')
        
        plt.legend(loc='upper right')


def on_key_press(event, system):
    """Handle keyboard input for formation switching"""
    if event.key == ' ':  # Spacebar
        system.switch_formation()
        print(f"🔄 Switched to {system.current_formation.upper()} formation")

def main():
    """Main function"""
    print("🚀 Starting Multi-Robot Formation Demo with Formation Switching...")
    print("🔴 Red star = Leader")
    print("🔵 Blue circle = Follower 1")
    print("🟢 Green square = Follower 2")  
    print("🟠 Orange triangle = Follower 3")
    print("❌ X marks = Target formation positions")
    print()
    print("🎮 CONTROLS:")
    print("   SPACEBAR = Switch formation patterns")
    print("   Ctrl+C = Stop demo")
    print()
    print("📋 Available formations: TRIANGLE → LINE → CIRCLE → V-SHAPE → (repeat)")
    print()
    
    # Create simulation
    system = MultiRobotSystem()
    
    # Create and run animation
    fig = plt.figure(figsize=(12, 10))
    ani = animation.FuncAnimation(fig, system.animate, interval=100, cache_frame_data=False)
    
    # Connect keyboard handler
    fig.canvas.mpl_connect('key_press_event', lambda event: on_key_press(event, system))
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Stopping multi-robot demo...")


if __name__ == '__main__':
    main() 