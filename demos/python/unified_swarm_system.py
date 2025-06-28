#!/usr/bin/env python3
"""
Unified Multi-Robot Leader-Follower Swarm System

This is the comprehensive demo that combines ALL features:
- Formation Control: Triangle, Line, Circle, V-Shape
- Computer Vision: Leader detection and tracking
- Obstacle Avoidance: Static and dynamic obstacles  
- Multiple Controllers: Proportional, MPC, Reinforcement Learning
- Real-time Performance Monitoring
- ROS2 Integration Ready

Controls:
- SPACEBAR: Switch formation patterns
- V: Toggle vision/manual leader control
- O: Toggle obstacle avoidance
- C: Cycle through controllers (Proportional → MPC → RL)
- A: Add random obstacles
- R: Reset simulation
- P: Show performance metrics
- Q: Quit

Author: Modern Swarm Leader-Follower System
Phase: Unified Integration Demo
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import math
import sys
import os
from typing import List, Dict, Tuple, Optional
from enum import Enum
import random
from collections import deque

# Import shared core logic
from swarm_core import Robot, ProportionalController, get_formation_targets, VisionSystem, Obstacle, calculate_obstacle_avoidance, update_dynamic_obstacles, MultiRobotMPCController

# All classes are defined within this file - no external imports needed

# Try importing advanced dependencies
try:
    import casadi as ca
    CASADI_AVAILABLE = True
    MPC_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    MPC_AVAILABLE = False

# RL is simulated in the unified system, so always available
RL_AVAILABLE = True

class FormationManager:
    """Manages different formation patterns."""
    
    def __init__(self):
        self.formations = {
            "triangle": self._triangle_formation,
            "line": self._line_formation,
            "circle": self._circle_formation,
            "v_shape": self._v_shape_formation
        }
    
    def get_formation_targets(self, leader_pos: np.ndarray, formation_type: str) -> List[np.ndarray]:
        """Get target positions for given formation."""
        if formation_type in self.formations:
            return self.formations[formation_type](leader_pos)
        else:
            return self._triangle_formation(leader_pos)
    
    def _triangle_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Triangle formation behind leader."""
        offsets = [
            np.array([-2.5, -1.5]),  # Left rear
            np.array([-2.5, 1.5]),   # Right rear
            np.array([-4.0, 0.0])    # Center rear
        ]
        return [leader_pos + offset for offset in offsets]
    
    def _line_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Line formation behind leader."""
        offsets = [
            np.array([-2.0, 0.0]),
            np.array([-4.0, 0.0]),
            np.array([-6.0, 0.0])
        ]
        return [leader_pos + offset for offset in offsets]
    
    def _circle_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Circular formation around leader."""
        radius = 3.0
        offsets = []
        for i in range(3):
            angle = 2 * math.pi * i / 3 + math.pi
            offset = np.array([radius * math.cos(angle), radius * math.sin(angle)])
            offsets.append(offset)
        return [leader_pos + offset for offset in offsets]
    
    def _v_shape_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """V-shape formation behind leader."""
        offsets = [
            np.array([-2.5, -2.5]),  # Left wing
            np.array([-2.5, 2.5]),   # Right wing
            np.array([-5.0, 0.0])    # Center rear
        ]
        return [leader_pos + offset for offset in offsets]

class SimpleObstacleManager:
    """Simple obstacle management with avoidance."""
    
    def __init__(self):
        self.static_obstacles = []  # List of (x, y, radius)
        self.dynamic_obstacles = []  # List of [x, y, radius, vx, vy]
    
    def add_static_obstacle(self, x: float, y: float, radius: float):
        """Add a static obstacle."""
        self.static_obstacles.append((x, y, radius))
    
    def add_dynamic_obstacle(self, x: float, y: float, radius: float, vx: float, vy: float):
        """Add a dynamic obstacle."""
        self.dynamic_obstacles.append([x, y, radius, vx, vy])
    
    def update_dynamic_obstacles(self, dt: float):
        """Update positions of dynamic obstacles."""
        for obstacle in self.dynamic_obstacles:
            obstacle[0] += obstacle[3] * dt  # Update x
            obstacle[1] += obstacle[4] * dt  # Update y
            
            # Bounce off boundaries
            if abs(obstacle[0]) > 12:
                obstacle[3] *= -1
            if abs(obstacle[1]) > 12:
                obstacle[4] *= -1
    
    def apply_obstacle_avoidance(self, robot_state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """Apply obstacle avoidance to control commands."""
        x, y, theta, _, _ = robot_state
        avoidance_control = control.copy()
        
        # Check all obstacles
        all_obstacles = [(ox, oy, r) for ox, oy, r in self.static_obstacles]
        all_obstacles.extend([(ox, oy, r) for ox, oy, r, _, _ in self.dynamic_obstacles])
        
        for ox, oy, radius in all_obstacles:
            distance = math.sqrt((x - ox)**2 + (y - oy)**2)
            safety_distance = radius + 1.5
            
            if distance < safety_distance:
                # Compute repulsion force
                if distance > 0.01:
                    repulsion_x = (x - ox) / distance
                    repulsion_y = (y - oy) / distance
                    
                    # Modify control
                    repulsion_strength = (safety_distance - distance) / safety_distance
                    avoidance_control[0] *= (1 - repulsion_strength * 0.5)
                    
                    # Add turning away from obstacle
                    avoidance_control[1] += math.atan2(repulsion_y, repulsion_x) - theta
        
        return avoidance_control
    
    def draw_obstacles(self, ax):
        """Draw obstacles on the plot."""
        # Static obstacles
        for x, y, radius in self.static_obstacles:
            circle = patches.Circle((x, y), radius, color='red', alpha=0.6)
            ax.add_patch(circle)
        
        # Dynamic obstacles
        for x, y, radius, _, _ in self.dynamic_obstacles:
            circle = patches.Circle((x, y), radius, color='purple', alpha=0.6)
            ax.add_patch(circle)
    
    def clear_all_obstacles(self):
        """Clear all obstacles."""
        self.static_obstacles.clear()
        self.dynamic_obstacles.clear()

class PerformanceMonitor:
    """Monitor and display system performance."""
    
    def __init__(self):
        self.formation_errors = [[] for _ in range(3)]  # For 3 followers
        self.control_efforts = [[] for _ in range(3)]
        self.start_time = time.time()
    
    def record_formation_error(self, robot_idx: int, error: float):
        """Record formation error for a robot."""
        if 0 <= robot_idx < len(self.formation_errors):
            self.formation_errors[robot_idx].append(error)
            # Keep only recent data
            if len(self.formation_errors[robot_idx]) > 500:
                self.formation_errors[robot_idx] = self.formation_errors[robot_idx][-500:]
    
    def record_control_effort(self, robot_idx: int, effort: float):
        """Record control effort for a robot."""
        if 0 <= robot_idx < len(self.control_efforts):
            self.control_efforts[robot_idx].append(effort)
            if len(self.control_efforts[robot_idx]) > 500:
                self.control_efforts[robot_idx] = self.control_efforts[robot_idx][-500:]
    
    def get_average_formation_errors(self) -> List[float]:
        """Get average formation errors for all robots."""
        averages = []
        for errors in self.formation_errors:
            if errors:
                averages.append(sum(errors[-50:]) / min(len(errors), 50))
            else:
                averages.append(0.0)
        return averages
    
    def plot_metrics(self, ax):
        """Plot performance metrics."""
        ax.clear()
        ax.set_title('Performance Metrics', fontsize=12, fontweight='bold')
        
        colors = ['blue', 'green', 'orange']
        
        # Plot formation errors
        for i, (errors, color) in enumerate(zip(self.formation_errors, colors)):
            if errors:
                recent_errors = errors[-100:]
                x = range(len(recent_errors))
                ax.plot(x, recent_errors, color=color, alpha=0.7, 
                       label=f'Follower {i+1} Error')
        
        ax.set_xlabel('Time Steps')
        ax.set_ylabel('Formation Error (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add average error text
        avg_errors = self.get_average_formation_errors()
        avg_text = "\n".join([f"Avg {i+1}: {error:.3f}m" for i, error in enumerate(avg_errors)])
        ax.text(0.02, 0.98, avg_text, transform=ax.transAxes, 
               verticalalignment='top', fontsize=10,
               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    
    def reset(self):
        """Reset all metrics."""
        for errors in self.formation_errors:
            errors.clear()
        for efforts in self.control_efforts:
            efforts.clear()
        self.start_time = time.time()

class UnifiedSwarmSystem:
    """
    Unified Multi-Robot Leader-Follower System
    Integrates all features into one comprehensive demonstration
    """
    
    def __init__(self):
        print("🚀 Initializing Unified Swarm System...")
        self.dt = 0.03  # Faster time step
        self.time = 0.0
        self.running = True
        self.leader = Robot(0, 0, 0, robot_id=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(-2, -1.5, 0, robot_id=1, color='blue', marker='o', name='Follower_1'),
            Robot(-2, 1.5, 0, robot_id=2, color='green', marker='s', name='Follower_2'),
            Robot(-3, 0, 0, robot_id=3, color='orange', marker='^', name='Follower_3')
        ]
        self.current_formation = "triangle"
        # Controllers with higher gains for faster response
        self.prop_controller = ProportionalController(kp_distance=2.5, kp_angle=4.0)
        self.mpc_controller = MultiRobotMPCController(num_robots=3, formation_type="triangle")
        self.controllers = ["Proportional", "MPC"]  # Removed RL
        self.current_controller_idx = 0
        self.last_formation_switch = time.time()
        # Vision integration
        self.vision_enabled = False
        self.vision_system = VisionSystem()
        
        print(f"VisionSystem methods: {[method for method in dir(self.vision_system) if not method.startswith('_')]}")
        self.camera_image = None
        self.manual_leader_pos = np.array([0.0, 0.0])
        # Leader trajectory parameters - much faster
        self.leader_speed = 1.2  # Increased from 0.4
        self.leader_radius = 8.0
        self.leader_center = np.array([0.0, 0.0])
        # Obstacle integration
        self.obstacles_enabled = False
        self.robot_collision_avoidance = True  # Always enabled for safety
        self.static_obstacles = [
            Obstacle(x=3, y=2, radius=1.0, color='darkred', name='Static_1'),
            Obstacle(x=-6, y=-3, radius=1.5, color='darkred', name='Static_2'),
            Obstacle(x=1, y=-4, radius=0.8, color='darkred', name='Static_3')
        ]
        self.dynamic_obstacles = [
            Obstacle(x=-2, y=5, radius=0.8, vx=0.6, vy=-0.3, color='purple', is_dynamic=True, name='Dynamic_1'),
            Obstacle(x=6, y=-2, radius=1.0, vx=-0.45, vy=0.9, color='purple', is_dynamic=True, name='Dynamic_2')
        ]
        print("✅ Unified Swarm System Initialized!")
        print("🎮 Press H for help with controls")

    def update_leader(self):
        """Update leader position based on control mode."""
        # Leader always moves independently (circular trajectory)
        omega = self.leader_speed / self.leader_radius
        self.leader.x = self.leader_center[0] + self.leader_radius * math.cos(omega * self.time)
        self.leader.y = self.leader_center[1] + self.leader_radius * math.sin(omega * self.time)
        self.leader.theta = omega * self.time + math.pi/2
        
        # Apply robot-to-robot collision avoidance for leader
        if self.robot_collision_avoidance:
            leader_avoidance_x, leader_avoidance_y = self.calculate_leader_avoidance()
            self.leader.x += leader_avoidance_x * self.dt * 0.3
            self.leader.y += leader_avoidance_y * self.dt * 0.3
        
        # Update leader's internal state
        self.leader.update(self.dt)
        
        # Create camera image for vision system demonstration
        if self.vision_enabled:
            all_robots = [self.leader] + self.followers
            self.camera_image = self.vision_system.create_synthetic_image(all_robots)
            
            # Demonstrate vision detection (but don't use it to control leader)
            if self.camera_image is not None:
                detected_pos = self.vision_system.detect_leader(self.camera_image, target_color='red')
                if detected_pos is not None:
                    # Store detected position for visualization purposes
                    if not hasattr(self, 'detection_history'):
                        self.detection_history = []
                    self.detection_history.append(detected_pos)
                    if len(self.detection_history) > 5:
                        self.detection_history.pop(0)

    def calculate_leader_avoidance(self):
        """Calculate avoidance forces to prevent leader from colliding with followers"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        # Check collision with all followers
        for follower in self.followers:
            dx = self.leader.x - follower.x
            dy = self.leader.y - follower.y
            distance = math.sqrt(dx**2 + dy**2)
            min_distance = 1.2  # Minimum safe distance between robots
            
            if distance < min_distance and distance > 0.1:
                force_magnitude = 2.0 * (min_distance - distance) / distance
                avoidance_x += force_magnitude * dx
                avoidance_y += force_magnitude * dy
        
        # Limit avoidance force
        max_avoidance = 1.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y

    def update_followers(self):
        leader_pos = np.array([self.leader.x, self.leader.y])
        formation_targets = get_formation_targets(leader_pos, self.current_formation)
        
        if self.controllers[self.current_controller_idx] == "Proportional":
            for i, follower in enumerate(self.followers):
                target_pos = formation_targets[i]
                control = self.prop_controller.compute_control(follower.get_state(), target_pos)
                
                # Apply obstacle avoidance if enabled
                if self.obstacles_enabled:
                    all_obstacles = self.static_obstacles + self.dynamic_obstacles
                    avoidance_x, avoidance_y = calculate_obstacle_avoidance(follower, all_obstacles)
                    # Apply avoidance as position correction, not control modification
                    follower.x += avoidance_x * self.dt * 0.5
                    follower.y += avoidance_y * self.dt * 0.5
                
                # Apply robot-to-robot collision avoidance
                if self.robot_collision_avoidance:
                    robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(follower, i)
                    follower.x += robot_avoidance_x * self.dt * 0.3
                    follower.y += robot_avoidance_y * self.dt * 0.3
                
                follower.update(self.dt, control[0], control[1])
        
        elif self.controllers[self.current_controller_idx] == "MPC":
            # Collect all robot states for MPC
            robot_states = [f.get_state() for f in self.followers]
            controls = self.mpc_controller.compute_control(robot_states, formation_targets)
            
            for i, (follower, control) in enumerate(zip(self.followers, controls)):
                # Apply obstacle avoidance if enabled
                if self.obstacles_enabled:
                    all_obstacles = self.static_obstacles + self.dynamic_obstacles
                    avoidance_x, avoidance_y = calculate_obstacle_avoidance(follower, all_obstacles)
                    # Apply avoidance as position correction, not control modification
                    follower.x += avoidance_x * self.dt * 0.5
                    follower.y += avoidance_y * self.dt * 0.5
                
                # Apply robot-to-robot collision avoidance
                if self.robot_collision_avoidance:
                    robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(follower, i)
                    follower.x += robot_avoidance_x * self.dt * 0.3
                    follower.y += robot_avoidance_y * self.dt * 0.3
                
                follower.update(self.dt, control[0], control[1])

    def calculate_robot_avoidance(self, current_robot, robot_index):
        """Calculate avoidance forces to prevent robot-to-robot collisions"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        # Check collision with leader
        dx = current_robot.x - self.leader.x
        dy = current_robot.y - self.leader.y
        distance = math.sqrt(dx**2 + dy**2)
        min_distance = 1.2  # Minimum safe distance between robots
        
        if distance < min_distance and distance > 0.1:
            force_magnitude = 2.0 * (min_distance - distance) / distance
            avoidance_x += force_magnitude * dx
            avoidance_y += force_magnitude * dy
        
        # Check collision with other followers
        for j, other_robot in enumerate(self.followers):
            if j != robot_index:  # Don't check collision with self
                dx = current_robot.x - other_robot.x
                dy = current_robot.y - other_robot.y
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < min_distance and distance > 0.1:
                    force_magnitude = 2.0 * (min_distance - distance) / distance
                    avoidance_x += force_magnitude * dx
                    avoidance_y += force_magnitude * dy
        
        # Limit avoidance force
        max_avoidance = 1.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y

    def switch_formation(self):
        if time.time() - self.last_formation_switch > 1.0:
            formations = ["triangle", "line", "circle", "v_shape"]
            current_idx = formations.index(self.current_formation)
            self.current_formation = formations[(current_idx + 1) % len(formations)]
            print(f"🔄 Formation switched to: {self.current_formation}")
            self.last_formation_switch = time.time()

    def cycle_controller(self):
        self.current_controller_idx = (self.current_controller_idx + 1) % len(self.controllers)
        print(f"🎛️ Controller switched to: {self.controllers[self.current_controller_idx]}")

    def update(self):
        self.update_leader()
        self.update_followers()
        # Update dynamic obstacles
        if self.obstacles_enabled:
            update_dynamic_obstacles(self.dynamic_obstacles, self.dt)
        self.time += self.dt

    def toggle_obstacles(self):
        self.obstacles_enabled = not self.obstacles_enabled
        print(f"🚧 Obstacle avoidance: {'ON' if self.obstacles_enabled else 'OFF'}")

    def add_random_obstacles(self):
        import random
        for _ in range(2):
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            radius = random.uniform(0.5, 1.5)
            if random.random() < 0.7:
                self.static_obstacles.append(Obstacle(x, y, radius, color='darkred'))
            else:
                vx = random.uniform(-0.8, 0.8)  # Increased speed range
                vy = random.uniform(-0.8, 0.8)  # Increased speed range
                self.dynamic_obstacles.append(Obstacle(x, y, radius, vx, vy, color='purple', is_dynamic=True))
        print(f"🚧 Added 2 random obstacles")

    def run_demo(self):
        print("🚀 Starting Unified Swarm System Demo...")
        print("Press H for help with controls")
        import matplotlib.gridspec as gridspec
        plt.ion()
        fig = plt.figure(figsize=(16, 8))
        gs = gridspec.GridSpec(1, 2, width_ratios=[2, 1])
        ax_main = fig.add_subplot(gs[0, 0])
        ax_camera = fig.add_subplot(gs[0, 1])
        def on_key_press(event):
            if event.key == ' ':
                self.switch_formation()
            elif event.key == 'v':
                self.vision_enabled = not self.vision_enabled
                print(f"👁️ Vision tracking: {'ON' if self.vision_enabled else 'OFF'}")
            elif event.key == 'o':
                self.toggle_obstacles()
            elif event.key == 'a':
                self.add_random_obstacles()
            elif event.key == 'c':
                self.cycle_controller()
            elif event.key == 'r':
                self.robot_collision_avoidance = not self.robot_collision_avoidance
                print(f"🤖 Robot collision avoidance: {'ON' if self.robot_collision_avoidance else 'OFF'}")
            elif event.key == 'q':
                self.running = False
        fig.canvas.mpl_connect('key_press_event', on_key_press)
        frame_count = 0
        start_time = time.time()
        try:
            while self.running:
                self.update()
                if frame_count % 2 == 0:  # Update visualization more frequently
                    self._update_visualization(ax_main, ax_camera)
                    plt.pause(0.01)
                frame_count += 1
                time.sleep(max(0, self.dt - 0.01))
        except KeyboardInterrupt:
            self.running = False
            print("\n🛑 Demo stopped by user")
        finally:
            runtime = time.time() - start_time
            print(f"⏱️ Total runtime: {runtime:.1f}s")
            try:
                plt.close('all')
                plt.ioff()
            except:
                pass

    def _update_visualization(self, ax_main, ax_camera):
        ax_main.clear()
        ax_main.set_xlim(-15, 15)
        ax_main.set_ylim(-15, 15)
        ax_main.set_aspect('equal')
        ax_main.grid(True, alpha=0.3)
        
        # Draw obstacles if enabled
        if self.obstacles_enabled:
            for obstacle in self.static_obstacles:
                circle = patches.Circle((obstacle.x, obstacle.y), obstacle.radius, color=obstacle.color, alpha=0.6)
                ax_main.add_patch(circle)
            for obstacle in self.dynamic_obstacles:
                circle = patches.Circle((obstacle.x, obstacle.y), obstacle.radius, color=obstacle.color, alpha=0.6)
                ax_main.add_patch(circle)
        
        leader_circle = patches.Circle((self.leader.x, self.leader.y), 0.6, color=self.leader.color, alpha=0.8, label='Leader')
        ax_main.add_patch(leader_circle)
        if len(self.leader.position_history) > 1:
            positions = self.leader.position_history[-100:]
            xs, ys = zip(*positions)
            ax_main.plot(xs, ys, color=self.leader.color, alpha=0.3, linewidth=2)
        leader_pos = np.array([self.leader.x, self.leader.y])
        formation_targets = get_formation_targets(leader_pos, self.current_formation)
        for i, (follower, target) in enumerate(zip(self.followers, formation_targets)):
            robot_circle = patches.Circle((follower.x, follower.y), 0.5, color=follower.color, alpha=0.8, label=f'Follower {i+1}')
            ax_main.add_patch(robot_circle)
            if len(follower.position_history) > 1:
                positions = follower.position_history[-50:]
                xs, ys = zip(*positions)
                ax_main.plot(xs, ys, color=follower.color, alpha=0.3, linewidth=1)
            ax_main.plot(target[0], target[1], 'x', color=follower.color, markersize=10, alpha=0.7, markeredgewidth=2)
            ax_main.plot([follower.x, target[0]], [follower.y, target[1]], '--', color=follower.color, alpha=0.5)
        
        # Status text
        status_text = (
            f"Formation: {self.current_formation}\n"
            f"Controller: {self.controllers[self.current_controller_idx]}\n"
            f"Vision: {'ON' if self.vision_enabled else 'OFF'}\n"
            f"Obstacles: {'ON' if self.obstacles_enabled else 'OFF'}\n"
            f"Robot Collision: {'ON' if self.robot_collision_avoidance else 'OFF'}\n"
            f"Time: {self.time:.1f}s"
        )
        ax_main.text(0.02, 0.98, status_text, transform=ax_main.transAxes, fontsize=11, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        controls_text = "Press H for help | SPACE: Formation | V: Vision | O: Obstacles | A: Add Obstacles | C: Controller | R: Robot Collision | Q: Quit"
        ax_main.text(0.5, 0.02, controls_text, transform=ax_main.transAxes, fontsize=9, ha='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        ax_main.set_title('Unified Multi-Robot Leader-Follower System', fontsize=16, fontweight='bold')
        ax_main.legend(loc='upper right')
        # Camera view
        ax_camera.clear()
        ax_camera.set_title('Camera View (Vision System)', fontsize=12)
        ax_camera.axis('off')
        if self.camera_image is not None:
            import cv2
            ax_camera.imshow(cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB))
        plt.tight_layout()

def main():
    """Main function to run the unified swarm system."""
    print("🤖 Unified Multi-Robot Leader-Follower Swarm System")
    print("=" * 60)
    
    # Check if we're in the right environment
    try:
        # Test if we can import our modules
        sys.path.append('src')
        print("📦 Loading system components...")
        
        # Create and run the system
        system = UnifiedSwarmSystem()
        system.run_demo()
        
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"💥 System error: {e}")
        print("Make sure you're running from the project root with:")
        print("   source activate_swarm_ros2.sh && python unified_swarm_system.py")

if __name__ == "__main__":
    main() 