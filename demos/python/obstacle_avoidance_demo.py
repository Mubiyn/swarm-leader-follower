#!/usr/bin/env python3
"""
🚧 Obstacle Avoidance Leader-Follower Demo
Adds static and dynamic obstacles to formation control
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import time
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional
import math

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

class ObstacleAvoidanceController:
    """Multi-robot swarm controller with obstacle avoidance"""
    
    def __init__(self):
        # Initialize robots
        self.leader = Robot(x=0, y=0, theta=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(x=-2, y=-2, theta=0, color='blue', marker='o', name='Follower_1'),
            Robot(x=-2, y=2, theta=0, color='green', marker='s', name='Follower_2'),
            Robot(x=-4, y=0, theta=0, color='orange', marker='^', name='Follower_3')
        ]
        
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
        
        # Control parameters
        self.dt = 0.05
        self.leader_speed = 0.3
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
        self.formation_keys = list(self.formations.keys())
        self.formation_index = 0
        
        # Visualization
        self.setup_visualization()
        
        # Control thread
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        
        # Performance metrics
        self.collision_count = 0
        self.path_efficiency = 1.0
        
    def setup_visualization(self):
        """Setup matplotlib visualization"""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(14, 10))
        
        self.ax.set_xlim(-12, 8)
        self.ax.set_ylim(-8, 8)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('🚧 Obstacle Avoidance Leader-Follower System', fontsize=16, fontweight='bold')
        
        # Initialize trails
        self.trails = {robot.name: {'x': [], 'y': []} for robot in [self.leader] + self.followers}
        
    def update_leader_motion(self):
        """Update leader robot motion with obstacle avoidance"""
        t = time.time()
        
        # Base circular motion
        radius = 3.0
        base_x = radius * np.cos(self.leader_speed * t)
        base_y = radius * np.sin(self.leader_speed * t)
        base_theta = self.leader_speed * t + np.pi/2
        
        # Apply obstacle avoidance to leader
        all_obstacles = self.static_obstacles + self.dynamic_obstacles
        avoidance_x, avoidance_y = self.calculate_obstacle_avoidance(
            Robot(x=base_x, y=base_y, theta=base_theta), all_obstacles
        )
        
        # Combine base motion with avoidance
        self.leader.x = base_x + avoidance_x
        self.leader.y = base_y + avoidance_y
        self.leader.theta = base_theta
        
        # Update leader velocity for path efficiency calculation
        self.leader.vx = avoidance_x / self.dt if self.dt > 0 else 0
        self.leader.vy = avoidance_y / self.dt if self.dt > 0 else 0
        
    def update_dynamic_obstacles(self):
        """Update positions of dynamic obstacles"""
        for obstacle in self.dynamic_obstacles:
            # Update position
            obstacle.x += obstacle.vx * self.dt
            obstacle.y += obstacle.vy * self.dt
            
            # Bounce off boundaries
            if obstacle.x > 8 or obstacle.x < -12:
                obstacle.vx *= -1
            if obstacle.y > 8 or obstacle.y < -8:
                obstacle.vy *= -1
                
    def calculate_obstacle_avoidance(self, robot: Robot, obstacles: List[Obstacle]) -> Tuple[float, float]:
        """Calculate avoidance forces for obstacles"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for obstacle in obstacles:
            # Calculate distance to obstacle
            dx = robot.x - obstacle.x
            dy = robot.y - obstacle.y
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
    
    def get_formation_target(self, follower_idx: int) -> Tuple[float, float]:
        """Get target position for follower in current formation"""
        leader_x, leader_y = self.leader.x, self.leader.y
        leader_theta = self.leader.theta
        
        # Get formation offset
        formation_offsets = self.formations[self.current_formation]
        if follower_idx >= len(formation_offsets):
            return leader_x - 2.0, leader_y
            
        dx, dy = formation_offsets[follower_idx]
        
        # Rotate offset based on leader orientation
        cos_theta = np.cos(leader_theta)
        sin_theta = np.sin(leader_theta)
        
        target_x = leader_x + dx * cos_theta - dy * sin_theta
        target_y = leader_y + dx * sin_theta + dy * cos_theta
        
        return target_x, target_y
    
    def update_follower_control(self, follower: Robot, target_x: float, target_y: float):
        """Update follower control with obstacle avoidance"""
        # Calculate formation attraction
        error_x = target_x - follower.x
        error_y = target_y - follower.y  
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
        all_obstacles = self.static_obstacles + self.dynamic_obstacles
        avoidance_x, avoidance_y = self.calculate_obstacle_avoidance(follower, all_obstacles)
        
        # Calculate robot-robot avoidance forces
        robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(
            follower, [self.leader] + [f for f in self.followers if f.name != follower.name]
        )
        
        # Combine all forces
        total_vx = formation_x + avoidance_x + robot_avoidance_x
        total_vy = formation_y + avoidance_y + robot_avoidance_y
        
        # Limit velocities
        total_vx = np.clip(total_vx, -self.max_linear_vel, self.max_linear_vel)
        total_vy = np.clip(total_vy, -self.max_linear_vel, self.max_linear_vel)
        
        # Update robot state
        follower.vx = total_vx
        follower.vy = total_vy
        
        # Update position
        follower.x += follower.vx * self.dt
        follower.y += follower.vy * self.dt
        
        # Update orientation to face movement direction
        if abs(follower.vx) > 0.1 or abs(follower.vy) > 0.1:
            follower.theta = np.arctan2(follower.vy, follower.vx)
    
    def calculate_robot_avoidance(self, robot: Robot, other_robots: List[Robot]) -> Tuple[float, float]:
        """Calculate avoidance forces for other robots"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for other in other_robots:
            dx = robot.x - other.x
            dy = robot.y - other.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < self.robot_safety_distance and distance > 0.1:
                force_magnitude = 0.5 * (self.robot_safety_distance - distance) / distance
                avoidance_x += force_magnitude * dx
                avoidance_y += force_magnitude * dy
        
        return avoidance_x, avoidance_y
    
    def check_collisions(self):
        """Check for collisions and update metrics"""
        all_robots = [self.leader] + self.followers
        all_obstacles = self.static_obstacles + self.dynamic_obstacles
        
        # Check robot-obstacle collisions
        for robot in all_robots:
            for obstacle in all_obstacles:
                dx = robot.x - obstacle.x
                dy = robot.y - obstacle.y
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance < obstacle.radius + 0.3:  # Robot radius ~0.3
                    self.collision_count += 1
        
        # Check robot-robot collisions
        for i, robot1 in enumerate(all_robots):
            for robot2 in all_robots[i+1:]:
                dx = robot1.x - robot2.x
                dy = robot1.y - robot2.y
                distance = np.sqrt(dx**2 + dy**2)
                
                if distance < 0.6:  # Two robot radii
                    self.collision_count += 1
    
    def control_loop(self):
        """Main control loop"""
        while self.running:
            try:
                # Update leader motion
                self.update_leader_motion()
                
                # Update dynamic obstacles
                self.update_dynamic_obstacles()
                
                # Update followers
                for i, follower in enumerate(self.followers):
                    target_x, target_y = self.get_formation_target(i)
                    self.update_follower_control(follower, target_x, target_y)
                
                # Check collisions
                self.check_collisions()
                
                time.sleep(self.dt)
                
            except Exception as e:
                print(f"Control loop error: {e}")
                break
    
    def update_visualization(self, frame):
        """Update visualization"""
        self.ax.clear()
        self.ax.set_xlim(-12, 8)
        self.ax.set_ylim(-8, 8)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # Title with metrics
        self.ax.set_title(
            f'🚧 Obstacle Avoidance System | Formation: {self.current_formation.upper()} | Collisions: {self.collision_count}', 
            fontsize=14, fontweight='bold'
        )
        
        # Draw static obstacles
        for obstacle in self.static_obstacles:
            circle = plt.Circle((obstacle.x, obstacle.y), obstacle.radius, 
                              color=obstacle.color, alpha=0.7, zorder=2)
            self.ax.add_patch(circle)
            self.ax.text(obstacle.x, obstacle.y, 'S', ha='center', va='center', 
                        color='white', fontweight='bold', fontsize=8)
        
        # Draw dynamic obstacles
        for obstacle in self.dynamic_obstacles:
            circle = plt.Circle((obstacle.x, obstacle.y), obstacle.radius, 
                              color=obstacle.color, alpha=0.7, zorder=2)
            self.ax.add_patch(circle)
            
            # Draw velocity arrow
            arrow_length = 2.0
            end_x = obstacle.x + arrow_length * obstacle.vx
            end_y = obstacle.y + arrow_length * obstacle.vy
            self.ax.arrow(obstacle.x, obstacle.y, arrow_length * obstacle.vx, arrow_length * obstacle.vy,
                         head_width=0.2, head_length=0.3, fc=obstacle.color, ec=obstacle.color, alpha=0.8)
            
            self.ax.text(obstacle.x, obstacle.y, 'D', ha='center', va='center', 
                        color='white', fontweight='bold', fontsize=8)
        
        # Update trails
        all_robots = [self.leader] + self.followers
        for robot in all_robots:
            trail = self.trails[robot.name]
            trail['x'].append(robot.x)
            trail['y'].append(robot.y)
            
            # Limit trail length
            if len(trail['x']) > 150:
                trail['x'].pop(0)
                trail['y'].pop(0)
            
            # Plot trail
            if len(trail['x']) > 1:
                self.ax.plot(trail['x'], trail['y'], color=robot.color, alpha=0.4, linewidth=2)
        
        # Plot robots
        colors = {'red': '🔴', 'blue': '🔵', 'green': '🟢', 'orange': '🟠'}
        
        # Leader
        self.ax.scatter(self.leader.x, self.leader.y, c=self.leader.color, s=250, 
                       marker=self.leader.marker, edgecolors='black', linewidth=2, 
                       label=f'{colors[self.leader.color]} Leader', zorder=5)
        
        # Followers and targets
        for i, follower in enumerate(self.followers):
            # Plot follower
            self.ax.scatter(follower.x, follower.y, c=follower.color, s=200, 
                           marker=follower.marker, edgecolors='black', linewidth=2,
                           label=f'{colors[follower.color]} {follower.name}', zorder=5)
            
            # Plot target position
            target_x, target_y = self.get_formation_target(i)
            self.ax.scatter(target_x, target_y, c=follower.color, s=100, 
                           marker='x', alpha=0.7, zorder=3)
            
            # Draw safety circle
            safety_circle = plt.Circle((follower.x, follower.y), self.robot_safety_distance, 
                                     fill=False, linestyle='--', alpha=0.3, color=follower.color)
            self.ax.add_patch(safety_circle)
        
        # Add legend
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Add control instructions
        info_text = (
            "🎮 CONTROLS:\n"
            "   SPACEBAR = Switch Formation\n"
            "   Ctrl+C = Stop\n"
            f"📊 STATUS:\n"
            f"   Formation: {self.current_formation.upper()}\n"
            f"   Collisions: {self.collision_count}\n"
            f"   Dynamic Obstacles: {len(self.dynamic_obstacles)}\n"
            f"   Static Obstacles: {len(self.static_obstacles)}\n"
            "🚧 LEGEND:\n"
            "   Red circles = Static obstacles\n"
            "   Purple circles = Dynamic obstacles\n"
            "   Dashed circles = Safety zones\n"
            "   Trails = Robot paths"
        )
        self.ax.text(-11.5, -7, info_text, fontsize=9, 
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcyan", alpha=0.8))
    
    def on_key_press(self, event):
        """Handle key press events"""
        if event.key == ' ':  # Spacebar - switch formation
            self.formation_index = (self.formation_index + 1) % len(self.formation_keys)
            self.current_formation = self.formation_keys[self.formation_index]
            print(f"🔄 Switched to {self.current_formation.upper()} formation")
    
    def run(self):
        """Run the demo"""
        print("🚀 Starting Obstacle Avoidance Multi-Robot Demo...")
        print("🔴 Red star = Leader")
        print("🔵🟢🟠 Colored shapes = Followers")
        print("🔴 Red circles = Static obstacles")
        print("🟣 Purple circles = Dynamic obstacles (moving)")
        print("❌ X marks = Target formation positions")
        print("⭕ Dashed circles = Robot safety zones")
        print("🎮 CONTROLS:")
        print("   SPACEBAR = Switch formation patterns")
        print("   Ctrl+C = Stop demo")
        print("📋 Available formations: TRIANGLE → LINE → CIRCLE → V-SHAPE → (repeat)")
        print("🚧 Watch robots navigate around obstacles while maintaining formation!")
        
        # Connect key press handler
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Start control thread
        self.control_thread.start()
        
        # Start animation
        animation = FuncAnimation(self.fig, self.update_visualization, interval=50, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n🛑 Stopping obstacle avoidance demo...")
        finally:
            self.running = False

if __name__ == "__main__":
    try:
        controller = ObstacleAvoidanceController()
        controller.run()
    except KeyboardInterrupt:
        print("\n🛑 Demo stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc() 