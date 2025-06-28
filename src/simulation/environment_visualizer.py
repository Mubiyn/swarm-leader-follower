"""
Environment Visualizer for Advanced Simulation Environments

This module provides visualization capabilities for procedurally generated
simulation environments, integrating with the multi-robot leader-follower system.

Features:
- Real-time environment rendering
- Dynamic obstacle updates
- Robot trajectory visualization
- Interactive controls
- Performance metrics overlay

Author: Modern Swarm Leader-Follower System
Phase: 6 - Advanced Simulation Environments
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, Circle, Polygon
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider
import time
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass

from scene_generator import SceneGenerator, Environment, Obstacle, ObstacleType, EnvironmentType

class EnvironmentVisualizer:
    """
    Advanced visualizer for simulation environments and robot systems.
    
    Provides real-time rendering of generated environments with integrated
    robot visualization, performance metrics, and interactive controls.
    """
    
    def __init__(self, figsize: Tuple[float, float] = (16, 10)):
        """
        Initialize environment visualizer.
        
        Args:
            figsize: Figure size for the visualization
        """
        self.figsize = figsize
        self.fig = None
        self.ax_main = None
        self.ax_metrics = None
        
        # Environment and robot data
        self.current_environment = None
        self.robots = {}
        self.robot_trails = {}
        self.robot_colors = {
            'leader': 'red',
            'follower_1': 'blue', 
            'follower_2': 'green',
            'follower_3': 'orange'
        }
        
        # Performance tracking
        self.metrics_history = {
            'time': [],
            'formation_error': [],
            'collision_count': [],
            'speed': []
        }
        
        # Visualization settings
        self.show_trails = True
        self.show_target_formation = True
        self.show_metrics = True
        self.trail_length = 100
        
        # Animation control
        self.is_running = False
        self.frame_count = 0
        
        print("🎬 Environment Visualizer initialized!")
    
    def setup_visualization(self, environment: Environment):
        """
        Setup the visualization for a given environment.
        
        Args:
            environment: Environment to visualize
        """
        self.current_environment = environment
        
        # Create figure with subplots
        self.fig = plt.figure(figsize=self.figsize)
        
        # Main environment view
        self.ax_main = self.fig.add_subplot(121)
        self.ax_main.set_xlim(environment.bounds[0] - 2, environment.bounds[1] + 2)
        self.ax_main.set_ylim(environment.bounds[2] - 2, environment.bounds[3] + 2)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_title(f"🎮 {environment.name}", fontsize=14, fontweight='bold')
        self.ax_main.set_xlabel("X Position (m)")
        self.ax_main.set_ylabel("Y Position (m)")
        
        # Metrics panel
        self.ax_metrics = self.fig.add_subplot(122)
        self.ax_metrics.set_title("📊 Performance Metrics", fontsize=12, fontweight='bold')
        
        # Initialize robot trails
        for robot_id in self.robot_colors.keys():
            self.robot_trails[robot_id] = {'x': [], 'y': []}
        
        # Render initial environment
        self._render_environment()
        
        plt.tight_layout()
        print(f"✅ Visualization setup complete for: {environment.name}")
    
    def _render_environment(self):
        """Render the static environment elements."""
        if not self.current_environment:
            return
        
        # Render obstacles
        for obstacle in self.current_environment.obstacles:
            self._render_obstacle(obstacle)
        
        # Mark start and goal positions
        if self.current_environment.start_position:
            start_x, start_y = self.current_environment.start_position
            self.ax_main.plot(start_x, start_y, 'g*', markersize=15, 
                             label='Start Position', markeredgecolor='black')
        
        if self.current_environment.goal_position:
            goal_x, goal_y = self.current_environment.goal_position
            self.ax_main.plot(goal_x, goal_y, 'r*', markersize=15, 
                             label='Goal Position', markeredgecolor='black')
    
    def _render_obstacle(self, obstacle: Obstacle):
        """Render a single obstacle."""
        x, y = obstacle.position
        
        if obstacle.obstacle_type in [ObstacleType.STATIC_CIRCLE, ObstacleType.DYNAMIC_CIRCLE]:
            radius = obstacle.size[0]
            circle = Circle((x, y), radius, color=obstacle.color, alpha=0.7)
            self.ax_main.add_patch(circle)
            
            # Add velocity arrow for dynamic obstacles
            if obstacle.velocity != (0.0, 0.0):
                vx, vy = obstacle.velocity
                self.ax_main.arrow(x, y, vx * 3, vy * 3, head_width=0.3, 
                                  head_length=0.2, fc='purple', ec='purple')
        
        else:  # Rectangle obstacles
            width, height = obstacle.size
            rect = Rectangle((x - width/2, y - height/2), width, height, 
                           color=obstacle.color, alpha=0.7)
            self.ax_main.add_patch(rect)
            
            # Add velocity arrow for dynamic obstacles
            if obstacle.velocity != (0.0, 0.0):
                vx, vy = obstacle.velocity
                self.ax_main.arrow(x, y, vx * 3, vy * 3, head_width=0.3, 
                                  head_length=0.2, fc='purple', ec='purple')
    
    def update_robots(self, robot_states: Dict[str, Dict]):
        """
        Update robot positions and states.
        
        Args:
            robot_states: Dictionary with robot states
                         Format: {'robot_id': {'position': (x, y), 'orientation': theta, ...}}
        """
        self.robots = robot_states.copy()
        
        # Update trails
        for robot_id, state in robot_states.items():
            if robot_id in self.robot_trails:
                x, y = state['position']
                self.robot_trails[robot_id]['x'].append(x)
                self.robot_trails[robot_id]['y'].append(y)
                
                # Limit trail length
                if len(self.robot_trails[robot_id]['x']) > self.trail_length:
                    self.robot_trails[robot_id]['x'].pop(0)
                    self.robot_trails[robot_id]['y'].pop(0)
    
    def update_metrics(self, metrics: Dict[str, float]):
        """
        Update performance metrics.
        
        Args:
            metrics: Dictionary with current performance metrics
        """
        current_time = time.time()
        self.metrics_history['time'].append(current_time)
        
        for key, value in metrics.items():
            if key in self.metrics_history:
                self.metrics_history[key].append(value)
        
        # Limit history length
        max_history = 200
        for key in self.metrics_history:
            if len(self.metrics_history[key]) > max_history:
                self.metrics_history[key].pop(0)
    
    def render_frame(self, target_formation_positions: Optional[Dict[str, Tuple[float, float]]] = None):
        """
        Render a single frame of the visualization.
        
        Args:
            target_formation_positions: Target positions for formation visualization
        """
        if not self.current_environment:
            return
        
        # Clear and re-render
        self.ax_main.clear()
        self.ax_main.set_xlim(self.current_environment.bounds[0] - 2, 
                              self.current_environment.bounds[1] + 2)
        self.ax_main.set_ylim(self.current_environment.bounds[2] - 2, 
                              self.current_environment.bounds[3] + 2)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_title(f"🎮 {self.current_environment.name}", 
                              fontsize=14, fontweight='bold')
        
        # Re-render environment
        self._render_environment()
        
        # Render robot trails
        if self.show_trails:
            for robot_id, color in self.robot_colors.items():
                if robot_id in self.robot_trails:
                    trail = self.robot_trails[robot_id]
                    if len(trail['x']) > 1:
                        self.ax_main.plot(trail['x'], trail['y'], color=color, 
                                         alpha=0.6, linewidth=2, linestyle='--')
        
        # Render robots
        for robot_id, state in self.robots.items():
            if robot_id in self.robot_colors:
                x, y = state['position']
                color = self.robot_colors[robot_id]
                
                # Robot body
                if robot_id == 'leader':
                    marker = '*'
                    size = 200
                elif 'follower_1' in robot_id:
                    marker = 'o'
                    size = 150
                elif 'follower_2' in robot_id:
                    marker = 's'
                    size = 150
                elif 'follower_3' in robot_id:
                    marker = '^'
                    size = 150
                else:
                    marker = 'o'
                    size = 100
                
                self.ax_main.scatter(x, y, c=color, marker=marker, s=size, 
                                   edgecolors='black', linewidth=2, zorder=10)
                
                # Orientation arrow
                if 'orientation' in state:
                    theta = state['orientation']
                    arrow_length = 1.0
                    dx = arrow_length * np.cos(theta)
                    dy = arrow_length * np.sin(theta)
                    self.ax_main.arrow(x, y, dx, dy, head_width=0.3, head_length=0.2, 
                                      fc=color, ec='black', alpha=0.8, zorder=11)
        
        # Render target formation positions
        if self.show_target_formation and target_formation_positions:
            for robot_id, target_pos in target_formation_positions.items():
                if robot_id in self.robot_colors:
                    tx, ty = target_pos
                    color = self.robot_colors[robot_id]
                    self.ax_main.plot(tx, ty, 'x', color=color, markersize=10, 
                                     markeredgewidth=3, alpha=0.7, zorder=9)
        
        # Update metrics panel
        self._render_metrics()
        
        # Update frame counter
        self.frame_count += 1
        
        # Refresh display
        plt.draw()
        plt.pause(0.01)
    
    def _render_metrics(self):
        """Render the performance metrics panel."""
        self.ax_metrics.clear()
        self.ax_metrics.set_title("📊 Performance Metrics", fontsize=12, fontweight='bold')
        
        if len(self.metrics_history['time']) > 1:
            # Formation error plot
            if len(self.metrics_history['formation_error']) > 1:
                recent_error = self.metrics_history['formation_error'][-50:]
                self.ax_metrics.plot(recent_error, 'b-', linewidth=2, label='Formation Error')
                self.ax_metrics.set_ylabel('Formation Error (m)')
                self.ax_metrics.legend()
            
            # Show latest metrics as text
            latest_metrics = {}
            for key in ['formation_error', 'collision_count', 'speed']:
                if key in self.metrics_history and self.metrics_history[key]:
                    latest_metrics[key] = self.metrics_history[key][-1]
            
            # Display metrics text
            metrics_text = []
            for key, value in latest_metrics.items():
                display_name = key.replace('_', ' ').title()
                metrics_text.append(f"{display_name}: {value:.2f}")
            
            if metrics_text:
                self.ax_metrics.text(0.05, 0.95, '\n'.join(metrics_text), 
                                   transform=self.ax_metrics.transAxes, fontsize=10,
                                   verticalalignment='top')
    
    def update_dynamic_obstacles(self, dt: float):
        """Update dynamic obstacle positions."""
        if self.current_environment:
            scene_generator = SceneGenerator()
            scene_generator.update_dynamic_obstacles(self.current_environment, dt)
    
    def close(self):
        """Close the visualization."""
        if self.fig:
            plt.close(self.fig)
        print("🎬 Visualization closed")


# Test the visualizer
if __name__ == "__main__":
    print("🧪 Testing Environment Visualizer...")
    
    # Create scene generator and environment
    generator = SceneGenerator(seed=42)
    env = generator.generate_environment(EnvironmentType.OBSTACLE_COURSE, difficulty=3)
    
    # Create visualizer
    visualizer = EnvironmentVisualizer()
    visualizer.setup_visualization(env)
    
    # Test robot updates
    test_robots = {
        'leader': {'position': (-15, 0), 'orientation': 0},
        'follower_1': {'position': (-17, -2), 'orientation': 0},
        'follower_2': {'position': (-17, 2), 'orientation': 0},
        'follower_3': {'position': (-19, 0), 'orientation': 0}
    }
    
    visualizer.update_robots(test_robots)
    
    # Test metrics update
    test_metrics = {
        'formation_error': 0.5,
        'collision_count': 0,
        'speed': 0.8
    }
    
    visualizer.update_metrics(test_metrics)
    
    # Render frame
    target_positions = {
        'follower_1': (-15, -2),
        'follower_2': (-15, 2), 
        'follower_3': (-17, 0)
    }
    
    visualizer.render_frame(target_positions)
    
    print("✅ Visualizer test complete!")
    print("🎮 Close the window to continue...")
    
    # Keep window open
    plt.show() 