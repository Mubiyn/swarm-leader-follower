#!/usr/bin/env python3
"""
Performance Plotter for ROS2 Swarm System

Generates comprehensive plots and charts from collected data:
- Formation error over time
- Robot trajectories
- Obstacle avoidance performance
- Controller response times
- Vision system accuracy
- System performance metrics
- Energy efficiency
- Collision statistics
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd
import seaborn as sns
from datetime import datetime
import json
import os
from typing import Dict, List, Any, Optional
import math


class PerformancePlotter:
    """Comprehensive plotting system for swarm performance analysis"""
    
    def __init__(self, output_dir: str = "performance_plots"):
        self.output_dir = output_dir
        self.data = {
            'formation_errors': [],
            'robot_positions': {},
            'obstacle_avoidance': [],
            'controller_metrics': [],
            'vision_metrics': [],
            'system_performance': [],
            'collision_events': [],
            'energy_metrics': [],
            'timestamps': []
        }
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Set up plotting style
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        
    def add_formation_error(self, timestamp: float, robot_id: str, error: float):
        """Add formation error data point"""
        self.data['formation_errors'].append({
            'timestamp': timestamp,
            'robot_id': robot_id,
            'error': error
        })
        self.data['timestamps'].append(timestamp)
        
    def add_robot_position(self, timestamp: float, robot_id: str, x: float, y: float, theta: float):
        """Add robot position data point"""
        if robot_id not in self.data['robot_positions']:
            self.data['robot_positions'][robot_id] = []
        
        self.data['robot_positions'][robot_id].append({
            'timestamp': timestamp,
            'x': x,
            'y': y,
            'theta': theta
        })
        
    def add_obstacle_avoidance(self, timestamp: float, robot_id: str, distance: float, action_taken: str):
        """Add obstacle avoidance data point"""
        self.data['obstacle_avoidance'].append({
            'timestamp': timestamp,
            'robot_id': robot_id,
            'distance': distance,
            'action': action_taken
        })
        
    def add_controller_metric(self, timestamp: float, robot_id: str, metric_type: str, value: float):
        """Add controller performance metric"""
        self.data['controller_metrics'].append({
            'timestamp': timestamp,
            'robot_id': robot_id,
            'metric_type': metric_type,
            'value': value
        })
        
    def add_vision_metric(self, timestamp: float, accuracy: float, detection_count: int, false_positives: int):
        """Add vision system metric"""
        self.data['vision_metrics'].append({
            'timestamp': timestamp,
            'accuracy': accuracy,
            'detection_count': detection_count,
            'false_positives': false_positives
        })
        
    def add_system_performance(self, timestamp: float, cpu_usage: float, memory_usage: float, latency: float):
        """Add system performance metric"""
        self.data['system_performance'].append({
            'timestamp': timestamp,
            'cpu_usage': cpu_usage,
            'memory_usage': memory_usage,
            'latency': latency
        })
        
    def add_collision_event(self, timestamp: float, robot_id: str, collision_type: str, severity: str):
        """Add collision event"""
        self.data['collision_events'].append({
            'timestamp': timestamp,
            'robot_id': robot_id,
            'type': collision_type,
            'severity': severity
        })
        
    def add_energy_metric(self, timestamp: float, robot_id: str, energy_consumption: float, battery_level: float):
        """Add energy consumption metric"""
        self.data['energy_metrics'].append({
            'timestamp': timestamp,
            'robot_id': robot_id,
            'energy_consumption': energy_consumption,
            'battery_level': battery_level
        })
        
    def generate_all_plots(self):
        """Generate all performance plots and save to files"""
        print("📊 Generating comprehensive performance plots...")
        
        # Create figure with subplots
        fig, axes = plt.subplots(3, 3, figsize=(20, 15))
        fig.suptitle('Swarm System Performance Analysis', fontsize=16, fontweight='bold')
        
        # Plot 1: Formation Error Over Time
        self._plot_formation_errors(axes[0, 0])
        
        # Plot 2: Robot Trajectories
        self._plot_robot_trajectories(axes[0, 1])
        
        # Plot 3: Obstacle Avoidance Performance
        self._plot_obstacle_avoidance(axes[0, 2])
        
        # Plot 4: Controller Performance
        self._plot_controller_performance(axes[1, 0])
        
        # Plot 5: Vision System Accuracy
        self._plot_vision_accuracy(axes[1, 1])
        
        # Plot 6: System Performance
        self._plot_system_performance(axes[1, 2])
        
        # Plot 7: Energy Consumption
        self._plot_energy_consumption(axes[2, 0])
        
        # Plot 8: Collision Statistics
        self._plot_collision_statistics(axes[2, 1])
        
        # Plot 9: Performance Summary
        self._plot_performance_summary(axes[2, 2])
        
        plt.tight_layout()
        
        # Save the comprehensive plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"comprehensive_performance_{timestamp}.png"
        filepath = os.path.join(self.output_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"📊 Comprehensive plot saved: {filepath}")
        
        # Generate individual detailed plots
        self._generate_individual_plots()
        
        # Generate performance report
        self._generate_performance_report()
        
        plt.close('all')
        
    def _plot_formation_errors(self, ax):
        """Plot formation errors over time"""
        if not self.data['formation_errors']:
            ax.text(0.5, 0.5, 'No formation error data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Formation Errors Over Time')
            return
            
        df = pd.DataFrame(self.data['formation_errors'])
        
        for robot_id in df['robot_id'].unique():
            robot_data = df[df['robot_id'] == robot_id]
            ax.plot(robot_data['timestamp'], robot_data['error'], 
                   label=f'Robot {robot_id}', linewidth=2, alpha=0.8)
        
        ax.set_title('Formation Errors Over Time', fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Formation Error (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add statistics
        mean_error = df['error'].mean()
        max_error = df['error'].max()
        ax.text(0.02, 0.98, f'Mean: {mean_error:.3f}m\nMax: {max_error:.3f}m', 
               transform=ax.transAxes, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
    def _plot_robot_trajectories(self, ax):
        """Plot robot trajectories"""
        if not self.data['robot_positions']:
            ax.text(0.5, 0.5, 'No trajectory data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Robot Trajectories')
            return
            
        colors = plt.cm.Set3(np.linspace(0, 1, len(self.data['robot_positions'])))
        
        for i, (robot_id, positions) in enumerate(self.data['robot_positions'].items()):
            if positions:
                df = pd.DataFrame(positions)
                ax.plot(df['x'], df['y'], color=colors[i], label=f'Robot {robot_id}', 
                       linewidth=2, alpha=0.8)
                
                # Mark start and end points
                ax.scatter(df['x'].iloc[0], df['y'].iloc[0], color=colors[i], 
                          marker='o', s=100, label=f'Start {robot_id}')
                ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color=colors[i], 
                          marker='s', s=100, label=f'End {robot_id}')
        
        ax.set_title('Robot Trajectories', fontweight='bold')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
    def _plot_obstacle_avoidance(self, ax):
        """Plot obstacle avoidance performance"""
        if not self.data['obstacle_avoidance']:
            ax.text(0.5, 0.5, 'No obstacle avoidance data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Obstacle Avoidance Performance')
            return
            
        df = pd.DataFrame(self.data['obstacle_avoidance'])
        
        # Plot distance to obstacles over time
        for robot_id in df['robot_id'].unique():
            robot_data = df[df['robot_id'] == robot_id]
            ax.plot(robot_data['timestamp'], robot_data['distance'], 
                   label=f'Robot {robot_id}', linewidth=2, alpha=0.8)
        
        # Add safety threshold line
        safety_threshold = 1.5  # meters
        ax.axhline(y=safety_threshold, color='red', linestyle='--', 
                  label='Safety Threshold', alpha=0.7)
        
        ax.set_title('Obstacle Avoidance Performance', fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance to Obstacle (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add statistics
        min_distance = df['distance'].min()
        avg_distance = df['distance'].mean()
        ax.text(0.02, 0.98, f'Min Distance: {min_distance:.2f}m\nAvg Distance: {avg_distance:.2f}m', 
               transform=ax.transAxes, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
    def _plot_controller_performance(self, ax):
        """Plot controller performance metrics"""
        if not self.data['controller_metrics']:
            ax.text(0.5, 0.5, 'No controller data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Controller Performance')
            return
            
        df = pd.DataFrame(self.data['controller_metrics'])
        
        # Group by metric type and plot
        for metric_type in df['metric_type'].unique():
            metric_data = df[df['metric_type'] == metric_type]
            ax.plot(metric_data['timestamp'], metric_data['value'], 
                   label=metric_type, linewidth=2, alpha=0.8)
        
        ax.set_title('Controller Performance Metrics', fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Metric Value')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def _plot_vision_accuracy(self, ax):
        """Plot vision system accuracy"""
        if not self.data['vision_metrics']:
            ax.text(0.5, 0.5, 'No vision data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Vision System Accuracy')
            return
            
        df = pd.DataFrame(self.data['vision_metrics'])
        
        # Plot accuracy over time
        ax.plot(df['timestamp'], df['accuracy'], color='blue', linewidth=2, label='Accuracy')
        
        # Add detection count on secondary y-axis
        ax2 = ax.twinx()
        ax2.plot(df['timestamp'], df['detection_count'], color='green', 
                linewidth=2, alpha=0.7, label='Detection Count')
        
        ax.set_title('Vision System Performance', fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Accuracy (%)', color='blue')
        ax2.set_ylabel('Detection Count', color='green')
        ax.grid(True, alpha=0.3)
        
        # Add statistics
        avg_accuracy = df['accuracy'].mean()
        total_detections = df['detection_count'].sum()
        ax.text(0.02, 0.98, f'Avg Accuracy: {avg_accuracy:.1f}%\nTotal Detections: {total_detections}', 
               transform=ax.transAxes, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
    def _plot_system_performance(self, ax):
        """Plot system performance metrics"""
        if not self.data['system_performance']:
            ax.text(0.5, 0.5, 'No system data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('System Performance')
            return
            
        df = pd.DataFrame(self.data['system_performance'])
        
        # Plot CPU and memory usage
        ax.plot(df['timestamp'], df['cpu_usage'], color='red', linewidth=2, label='CPU Usage (%)')
        ax.plot(df['timestamp'], df['memory_usage'], color='blue', linewidth=2, label='Memory Usage (%)')
        
        # Add latency on secondary y-axis
        ax2 = ax.twinx()
        ax2.plot(df['timestamp'], df['latency'], color='green', 
                linewidth=2, alpha=0.7, label='Latency (ms)')
        
        ax.set_title('System Performance Metrics', fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Usage (%)', color='black')
        ax2.set_ylabel('Latency (ms)', color='green')
        ax.legend(loc='upper left')
        ax2.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
    def _plot_energy_consumption(self, ax):
        """Plot energy consumption metrics"""
        if not self.data['energy_metrics']:
            ax.text(0.5, 0.5, 'No energy data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Energy Consumption')
            return
            
        df = pd.DataFrame(self.data['energy_metrics'])
        
        # Plot energy consumption and battery level
        for robot_id in df['robot_id'].unique():
            robot_data = df[df['robot_id'] == robot_id]
            ax.plot(robot_data['timestamp'], robot_data['energy_consumption'], 
                   label=f'Energy {robot_id}', linewidth=2, alpha=0.8)
        
        # Add battery level on secondary y-axis
        ax2 = ax.twinx()
        for robot_id in df['robot_id'].unique():
            robot_data = df[df['robot_id'] == robot_id]
            ax2.plot(robot_data['timestamp'], robot_data['battery_level'], 
                    linestyle='--', alpha=0.7, label=f'Battery {robot_id}')
        
        ax.set_title('Energy Consumption & Battery Level', fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Energy Consumption (J)', color='black')
        ax2.set_ylabel('Battery Level (%)', color='green')
        ax.legend(loc='upper left')
        ax2.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
    def _plot_collision_statistics(self, ax):
        """Plot collision statistics"""
        if not self.data['collision_events']:
            ax.text(0.5, 0.5, 'No collision data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Collision Statistics')
            return
            
        df = pd.DataFrame(self.data['collision_events'])
        
        # Count collisions by type and severity
        collision_counts = df.groupby(['type', 'severity']).size().unstack(fill_value=0)
        
        if not collision_counts.empty:
            collision_counts.plot(kind='bar', ax=ax, alpha=0.8)
            ax.set_title('Collision Statistics by Type and Severity', fontweight='bold')
            ax.set_xlabel('Collision Type')
            ax.set_ylabel('Number of Collisions')
            ax.legend(title='Severity')
            ax.tick_params(axis='x', rotation=45)
        else:
            ax.text(0.5, 0.5, 'No collisions recorded', ha='center', va='center', transform=ax.transAxes)
            ax.set_title('Collision Statistics')
        
        ax.grid(True, alpha=0.3)
        
    def _plot_performance_summary(self, ax):
        """Plot performance summary dashboard"""
        # Create a summary table
        summary_data = []
        
        # Formation performance
        if self.data['formation_errors']:
            df_formation = pd.DataFrame(self.data['formation_errors'])
            summary_data.append(['Formation Error', f"{df_formation['error'].mean():.3f}m", f"{df_formation['error'].max():.3f}m"])
        
        # Obstacle avoidance
        if self.data['obstacle_avoidance']:
            df_obstacle = pd.DataFrame(self.data['obstacle_avoidance'])
            summary_data.append(['Min Obstacle Dist', f"{df_obstacle['distance'].min():.2f}m", f"{df_obstacle['distance'].mean():.2f}m"])
        
        # Vision accuracy
        if self.data['vision_metrics']:
            df_vision = pd.DataFrame(self.data['vision_metrics'])
            summary_data.append(['Vision Accuracy', f"{df_vision['accuracy'].mean():.1f}%", f"{df_vision['detection_count'].sum()} detections"])
        
        # System performance
        if self.data['system_performance']:
            df_system = pd.DataFrame(self.data['system_performance'])
            summary_data.append(['Avg CPU Usage', f"{df_system['cpu_usage'].mean():.1f}%", f"{df_system['latency'].mean():.1f}ms latency"])
        
        if summary_data:
            table = ax.table(cellText=summary_data,
                           colLabels=['Metric', 'Average', 'Peak/Total'],
                           cellLoc='center',
                           loc='center')
            table.auto_set_font_size(False)
            table.set_fontsize(10)
            table.scale(1, 2)
            
            # Style the table
            for i in range(len(summary_data) + 1):
                for j in range(3):
                    if i == 0:  # Header row
                        table[(i, j)].set_facecolor('#4CAF50')
                        table[(i, j)].set_text_props(weight='bold', color='white')
                    else:
                        table[(i, j)].set_facecolor('#f0f0f0')
        
        ax.set_title('Performance Summary', fontweight='bold')
        ax.axis('off')
        
    def _generate_individual_plots(self):
        """Generate individual detailed plots"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. Formation Error Analysis
        if self.data['formation_errors']:
            self._generate_formation_analysis(timestamp)
        
        # 2. Trajectory Analysis
        if self.data['robot_positions']:
            self._generate_trajectory_analysis(timestamp)
        
        # 3. Obstacle Avoidance Analysis
        if self.data['obstacle_avoidance']:
            self._generate_obstacle_analysis(timestamp)
        
        # 4. System Performance Analysis
        if self.data['system_performance']:
            self._generate_system_analysis(timestamp)
        
    def _generate_formation_analysis(self, timestamp):
        """Generate detailed formation error analysis"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        df = pd.DataFrame(self.data['formation_errors'])
        
        # Time series plot
        for robot_id in df['robot_id'].unique():
            robot_data = df[df['robot_id'] == robot_id]
            ax1.plot(robot_data['timestamp'], robot_data['error'], label=f'Robot {robot_id}')
        
        ax1.set_title('Formation Error Over Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Error (m)')
        ax1.legend()
        ax1.grid(True)
        
        # Histogram
        ax2.hist(df['error'], bins=20, alpha=0.7, edgecolor='black')
        ax2.set_title('Formation Error Distribution')
        ax2.set_xlabel('Error (m)')
        ax2.set_ylabel('Frequency')
        ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, f'formation_analysis_{timestamp}.png'), dpi=300)
        plt.close()
        
    def _generate_trajectory_analysis(self, timestamp):
        """Generate detailed trajectory analysis"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        colors = plt.cm.Set3(np.linspace(0, 1, len(self.data['robot_positions'])))
        
        # Trajectory plot
        for i, (robot_id, positions) in enumerate(self.data['robot_positions'].items()):
            if positions:
                df = pd.DataFrame(positions)
                ax1.plot(df['x'], df['y'], color=colors[i], label=f'Robot {robot_id}', linewidth=2)
                ax1.scatter(df['x'].iloc[0], df['y'].iloc[0], color=colors[i], marker='o', s=100)
                ax1.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color=colors[i], marker='s', s=100)
        
        ax1.set_title('Robot Trajectories')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.legend()
        ax1.grid(True)
        ax1.set_aspect('equal')
        
        # Distance traveled
        distances = []
        robot_ids = []
        for robot_id, positions in self.data['robot_positions'].items():
            if len(positions) > 1:
                df = pd.DataFrame(positions)
                total_distance = 0
                for j in range(1, len(df)):
                    dx = df.iloc[j]['x'] - df.iloc[j-1]['x']
                    dy = df.iloc[j]['y'] - df.iloc[j-1]['y']
                    total_distance += math.sqrt(dx*dx + dy*dy)
                distances.append(total_distance)
                robot_ids.append(robot_id)
        
        if distances:
            ax2.bar(robot_ids, distances, alpha=0.7)
            ax2.set_title('Total Distance Traveled')
            ax2.set_xlabel('Robot ID')
            ax2.set_ylabel('Distance (m)')
            ax2.tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, f'trajectory_analysis_{timestamp}.png'), dpi=300)
        plt.close()
        
    def _generate_obstacle_analysis(self, timestamp):
        """Generate detailed obstacle avoidance analysis"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        df = pd.DataFrame(self.data['obstacle_avoidance'])
        
        # Distance over time
        for robot_id in df['robot_id'].unique():
            robot_data = df[df['robot_id'] == robot_id]
            ax1.plot(robot_data['timestamp'], robot_data['distance'], label=f'Robot {robot_id}')
        
        ax1.axhline(y=1.5, color='red', linestyle='--', label='Safety Threshold')
        ax1.set_title('Distance to Obstacles Over Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance (m)')
        ax1.legend()
        ax1.grid(True)
        
        # Action distribution
        action_counts = df['action'].value_counts()
        ax2.pie(action_counts.values, labels=action_counts.index, autopct='%1.1f%%')
        ax2.set_title('Obstacle Avoidance Actions')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, f'obstacle_analysis_{timestamp}.png'), dpi=300)
        plt.close()
        
    def _generate_system_analysis(self, timestamp):
        """Generate detailed system performance analysis"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        df = pd.DataFrame(self.data['system_performance'])
        
        # CPU usage
        ax1.plot(df['timestamp'], df['cpu_usage'], color='red')
        ax1.set_title('CPU Usage Over Time')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('CPU Usage (%)')
        ax1.grid(True)
        
        # Memory usage
        ax2.plot(df['timestamp'], df['memory_usage'], color='blue')
        ax2.set_title('Memory Usage Over Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Memory Usage (%)')
        ax2.grid(True)
        
        # Latency
        ax3.plot(df['timestamp'], df['latency'], color='green')
        ax3.set_title('System Latency Over Time')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Latency (ms)')
        ax3.grid(True)
        
        # Performance correlation
        ax4.scatter(df['cpu_usage'], df['latency'], alpha=0.6)
        ax4.set_title('CPU Usage vs Latency')
        ax4.set_xlabel('CPU Usage (%)')
        ax4.set_ylabel('Latency (ms)')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, f'system_analysis_{timestamp}.png'), dpi=300)
        plt.close()
        
    def _generate_performance_report(self):
        """Generate a comprehensive performance report"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = os.path.join(self.output_dir, f'performance_report_{timestamp}.txt')
        
        with open(report_file, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("SWARM SYSTEM PERFORMANCE REPORT\n")
            f.write("=" * 60 + "\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            # Formation Performance
            if self.data['formation_errors']:
                df_formation = pd.DataFrame(self.data['formation_errors'])
                f.write("FORMATION PERFORMANCE:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Average Error: {df_formation['error'].mean():.3f}m\n")
                f.write(f"Maximum Error: {df_formation['error'].max():.3f}m\n")
                f.write(f"Standard Deviation: {df_formation['error'].std():.3f}m\n")
                f.write(f"Total Measurements: {len(df_formation)}\n\n")
            
            # Obstacle Avoidance
            if self.data['obstacle_avoidance']:
                df_obstacle = pd.DataFrame(self.data['obstacle_avoidance'])
                f.write("OBSTACLE AVOIDANCE:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Minimum Distance: {df_obstacle['distance'].min():.2f}m\n")
                f.write(f"Average Distance: {df_obstacle['distance'].mean():.2f}m\n")
                f.write(f"Safety Violations: {len(df_obstacle[df_obstacle['distance'] < 1.5])}\n")
                f.write(f"Total Avoidance Events: {len(df_obstacle)}\n\n")
            
            # Vision Performance
            if self.data['vision_metrics']:
                df_vision = pd.DataFrame(self.data['vision_metrics'])
                f.write("VISION SYSTEM:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Average Accuracy: {df_vision['accuracy'].mean():.1f}%\n")
                f.write(f"Total Detections: {df_vision['detection_count'].sum()}\n")
                f.write(f"False Positives: {df_vision['false_positives'].sum()}\n")
                f.write(f"Detection Rate: {df_vision['detection_count'].mean():.1f} per measurement\n\n")
            
            # System Performance
            if self.data['system_performance']:
                df_system = pd.DataFrame(self.data['system_performance'])
                f.write("SYSTEM PERFORMANCE:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Average CPU Usage: {df_system['cpu_usage'].mean():.1f}%\n")
                f.write(f"Average Memory Usage: {df_system['memory_usage'].mean():.1f}%\n")
                f.write(f"Average Latency: {df_system['latency'].mean():.1f}ms\n")
                f.write(f"Peak CPU Usage: {df_system['cpu_usage'].max():.1f}%\n")
                f.write(f"Peak Memory Usage: {df_system['memory_usage'].max():.1f}%\n\n")
            
            # Collision Statistics
            if self.data['collision_events']:
                df_collision = pd.DataFrame(self.data['collision_events'])
                f.write("COLLISION STATISTICS:\n")
                f.write("-" * 30 + "\n")
                f.write(f"Total Collisions: {len(df_collision)}\n")
                collision_by_type = df_collision['type'].value_counts()
                for collision_type, count in collision_by_type.items():
                    f.write(f"{collision_type}: {count}\n")
                f.write("\n")
            
            # Energy Consumption
            if self.data['energy_metrics']:
                df_energy = pd.DataFrame(self.data['energy_metrics'])
                f.write("ENERGY CONSUMPTION:\n")
                f.write("-" * 30 + "\n")
                for robot_id in df_energy['robot_id'].unique():
                    robot_energy = df_energy[df_energy['robot_id'] == robot_id]
                    f.write(f"Robot {robot_id}:\n")
                    f.write(f"  Total Energy: {robot_energy['energy_consumption'].sum():.1f}J\n")
                    f.write(f"  Average Battery: {robot_energy['battery_level'].mean():.1f}%\n")
                    f.write(f"  Final Battery: {robot_energy['battery_level'].iloc[-1]:.1f}%\n")
                f.write("\n")
            
            # Overall Assessment
            f.write("OVERALL ASSESSMENT:\n")
            f.write("-" * 30 + "\n")
            
            # Calculate overall score
            score = 100
            issues = []
            
            if self.data['formation_errors']:
                avg_error = df_formation['error'].mean()
                if avg_error > 2.0:
                    score -= 20
                    issues.append("High formation error")
                elif avg_error > 1.0:
                    score -= 10
                    issues.append("Moderate formation error")
            
            if self.data['obstacle_avoidance']:
                safety_violations = len(df_obstacle[df_obstacle['distance'] < 1.5])
                if safety_violations > 10:
                    score -= 15
                    issues.append("Multiple safety violations")
                elif safety_violations > 5:
                    score -= 10
                    issues.append("Some safety violations")
            
            if self.data['system_performance']:
                avg_cpu = df_system['cpu_usage'].mean()
                if avg_cpu > 80:
                    score -= 10
                    issues.append("High CPU usage")
                elif avg_cpu > 60:
                    score -= 5
                    issues.append("Moderate CPU usage")
            
            f.write(f"Overall Performance Score: {score}/100\n")
            if issues:
                f.write("Issues Identified:\n")
                for issue in issues:
                    f.write(f"  - {issue}\n")
            else:
                f.write("No significant issues identified.\n")
            
            f.write("\n" + "=" * 60 + "\n")
            f.write("END OF REPORT\n")
            f.write("=" * 60 + "\n")
        
        print(f"📄 Performance report saved: {report_file}")


if __name__ == "__main__":
    # Example usage
    plotter = PerformancePlotter()
    
    # Add some sample data
    for i in range(100):
        t = i * 0.1
        plotter.add_formation_error(t, "robot_1", 0.5 + 0.2 * np.sin(t))
        plotter.add_formation_error(t, "robot_2", 0.3 + 0.1 * np.cos(t))
        plotter.add_robot_position(t, "robot_1", np.cos(t), np.sin(t), t)
        plotter.add_robot_position(t, "robot_2", 2*np.cos(t), 2*np.sin(t), t)
        plotter.add_obstacle_avoidance(t, "robot_1", 2.0 + 0.5*np.sin(t), "avoid")
        plotter.add_system_performance(t, 30 + 10*np.sin(t), 50 + 5*np.cos(t), 5 + 2*np.random.random())
    
    # Generate plots
    plotter.generate_all_plots() 