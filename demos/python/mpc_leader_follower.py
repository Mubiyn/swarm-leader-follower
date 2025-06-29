"""
MPC-Based Multi-Robot Leader-Follower System

This demo integrates Model Predictive Control (MPC) with the multi-robot formation system.
Features:
- MPC-based formation control vs. traditional proportional control
- Real-time performance comparison
- Formation switching with optimized control
- Live metrics and visualization

Controls:
- SPACEBAR: Switch formation patterns  
- M: Toggle between MPC and Proportional control
- P: Show performance metrics
- Ctrl+C: Stop demo

Author: Modern Swarm Leader-Follower System  
Phase: 5 - Advanced Control (MPC)
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import math
from typing import List, Dict, Tuple
import sys
import os

# Add the src directory to path for imports
# sys.path.append('src')  # No longer needed since file is in same directory
from mpc_controller import MultiRobotMPCController

class Robot:
    """Robot class with state management."""
    def __init__(self, x: float, y: float, theta: float = 0.0, robot_id: int = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.robot_id = robot_id
        
        # Control history for analysis
        self.position_history = [(x, y)]
        self.control_history = []
        
        # Performance metrics
        self.formation_errors = []
        self.control_efforts = []
    
    def get_state(self) -> np.ndarray:
        """Get robot state as numpy array."""
        return np.array([self.x, self.y, self.theta, self.v, self.omega])
    
    def update(self, dt: float, v_cmd: float = None, omega_cmd: float = None):
        """Update robot state with control inputs."""
        if v_cmd is not None and omega_cmd is not None:
            self.v = v_cmd
            self.omega = omega_cmd
            self.control_history.append((v_cmd, omega_cmd))
        
        # Update position using bicycle model
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Store position history
        self.position_history.append((self.x, self.y))
        
        # Limit history length for performance
        if len(self.position_history) > 500:
            self.position_history.pop(0)
        if len(self.control_history) > 500:
            self.control_history.pop(0)

class ProportionalController:
    """Traditional proportional controller for comparison."""
    
    def __init__(self, kp_distance: float = 1.0, kp_angle: float = 2.0):
        self.kp_distance = kp_distance
        self.kp_angle = kp_angle
    
    def compute_control(self, robot_state: np.ndarray, target_pos: np.ndarray) -> np.ndarray:
        """Compute proportional control."""
        x, y, theta, _, _ = robot_state
        target_x, target_y = target_pos
        
        # Distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Angle error
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Control commands
        v_cmd = min(self.kp_distance * distance, 1.0)  # Max speed limit
        omega_cmd = max(min(self.kp_angle * angle_error, 1.5), -1.5)  # Max angular speed
        
        return np.array([v_cmd, omega_cmd])

class MPCLeaderFollowerSystem:
    """MPC-based multi-robot leader-follower system with performance comparison."""
    
    def __init__(self):
        # Initialize robots (1 leader + 3 followers)
        self.leader = Robot(0, 0, 0, robot_id=0)
        self.followers = [
            Robot(-2, -1.5, 0, robot_id=1),
            Robot(-2, 1.5, 0, robot_id=2), 
            Robot(-3, 0, 0, robot_id=3)
        ]
        
        # Controllers
        self.mpc_controller = MultiRobotMPCController(
            num_robots=4,
            formation_type="triangle",
            prediction_horizon=8,
            dt=0.05,
            max_linear_vel=1.0,
            max_angular_vel=1.5,
            formation_weight=10.0,
            control_weight=0.1
        )
        
        self.prop_controller = ProportionalController(kp_distance=1.0, kp_angle=2.0)
        
        # Formation patterns
        self.formations = ["triangle", "line", "circle", "v_shape"]
        self.current_formation_idx = 0
        
        # Control mode
        self.use_mpc = True
        self.last_formation_switch = time.time()
        
        # Performance tracking
        self.mpc_metrics = {
            'solve_times': [],
            'costs': [],
            'formation_errors': [],
            'control_efforts': []
        }
        self.prop_metrics = {
            'formation_errors': [],
            'control_efforts': []
        }
        
        # Simulation parameters
        self.dt = 0.05
        self.time = 0.0
        
        # Leader trajectory parameters
        self.leader_speed = 0.3
        self.leader_radius = 8.0
        
        print(f"🚀 MPC Leader-Follower System Initialized!")
        print(f"🎮 Controls:")
        print(f"   SPACEBAR: Switch formations")
        print(f"   M: Toggle MPC/Proportional control")
        print(f"   P: Show performance metrics")
    
    def get_formation_targets(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Get formation target positions."""
        formation_type = self.formations[self.current_formation_idx]
        
        if formation_type == "triangle":
            offsets = [
                np.array([-2.0, -1.5]),  # Left rear
                np.array([-2.0, 1.5]),   # Right rear
                np.array([-3.0, 0.0])    # Center rear
            ]
        elif formation_type == "line":
            offsets = [
                np.array([-2.0, 0.0]),
                np.array([-4.0, 0.0]),
                np.array([-6.0, 0.0])
            ]
        elif formation_type == "circle":
            radius = 2.5
            offsets = []
            for i in range(3):
                angle = 2 * math.pi * i / 3 + math.pi
                offset = np.array([radius * math.cos(angle), radius * math.sin(angle)])
                offsets.append(offset)
        else:  # v_shape
            offsets = [
                np.array([-2.0, -2.0]),  # Left wing
                np.array([-2.0, 2.0]),   # Right wing
                np.array([-4.0, 0.0])    # Center rear
            ]
        
        targets = []
        for offset in offsets:
            target = leader_pos + offset
            targets.append(target)
        
        return targets
    
    def update_leader(self):
        """Update leader position (circular trajectory)."""
        self.leader.x = self.leader_radius * math.cos(self.leader_speed * self.time)
        self.leader.y = self.leader_radius * math.sin(self.leader_speed * self.time)
        self.leader.theta = self.leader_speed * self.time + math.pi/2
        
        # Update leader history
        self.leader.position_history.append((self.leader.x, self.leader.y))
        if len(self.leader.position_history) > 500:
            self.leader.position_history.pop(0)
    
    def update_followers(self):
        """Update followers using current controller."""
        leader_pos = np.array([self.leader.x, self.leader.y])
        targets = self.get_formation_targets(leader_pos)
        
        if self.use_mpc:
            # Use MPC controller
            robot_states = [follower.get_state() for follower in self.followers]
            
            # Set formation type in MPC controller
            formation_name = self.formations[self.current_formation_idx]
            self.mpc_controller.set_formation_type(formation_name)
            
            # Compute MPC controls
            results = self.mpc_controller.compute_formation_controls(robot_states, leader_pos)
            
            # Apply controls and track metrics
            total_formation_error = 0
            total_control_effort = 0
            
            for i, (follower, (control, info)) in enumerate(zip(self.followers, results)):
                v_cmd, omega_cmd = control
                follower.update(self.dt, v_cmd, omega_cmd)
                
                # Track formation error
                target = targets[i]
                error = np.linalg.norm([follower.x - target[0], follower.y - target[1]])
                follower.formation_errors.append(error)
                total_formation_error += error
                
                # Track control effort
                effort = v_cmd**2 + omega_cmd**2
                follower.control_efforts.append(effort)
                total_control_effort += effort
                
                # Track MPC-specific metrics
                if info['success']:
                    self.mpc_metrics['solve_times'].append(info['solve_time'])
                    self.mpc_metrics['costs'].append(info['cost'])
            
            self.mpc_metrics['formation_errors'].append(total_formation_error / len(self.followers))
            self.mpc_metrics['control_efforts'].append(total_control_effort / len(self.followers))
            
        else:
            # Use proportional controller
            total_formation_error = 0
            total_control_effort = 0
            
            for i, (follower, target) in enumerate(zip(self.followers, targets)):
                robot_state = follower.get_state()
                control = self.prop_controller.compute_control(robot_state, target)
                v_cmd, omega_cmd = control
                follower.update(self.dt, v_cmd, omega_cmd)
                
                # Track formation error
                error = np.linalg.norm([follower.x - target[0], follower.y - target[1]])
                follower.formation_errors.append(error)
                total_formation_error += error
                
                # Track control effort
                effort = v_cmd**2 + omega_cmd**2
                follower.control_efforts.append(effort)
                total_control_effort += effort
            
            self.prop_metrics['formation_errors'].append(total_formation_error / len(self.followers))
            self.prop_metrics['control_efforts'].append(total_control_effort / len(self.followers))
    
    def switch_formation(self):
        """Switch to next formation pattern."""
        self.current_formation_idx = (self.current_formation_idx + 1) % len(self.formations)
        formation_name = self.formations[self.current_formation_idx]
        print(f"🔄 Switched to {formation_name.upper()} formation")
        self.last_formation_switch = time.time()
    
    def toggle_controller(self):
        """Toggle between MPC and proportional controller."""
        self.use_mpc = not self.use_mpc
        controller_name = "MPC" if self.use_mpc else "Proportional"
        print(f"🔄 Switched to {controller_name} controller")
    
    def show_performance_metrics(self):
        """Display performance metrics comparison."""
        print("\n📊 PERFORMANCE METRICS COMPARISON")
        print("=" * 50)
        
        if self.mpc_metrics['formation_errors']:
            mpc_avg_error = np.mean(self.mpc_metrics['formation_errors'][-100:])
            mpc_avg_effort = np.mean(self.mpc_metrics['control_efforts'][-100:])
            mpc_avg_solve_time = np.mean(self.mpc_metrics['solve_times'][-100:]) if self.mpc_metrics['solve_times'] else 0
            
            print(f"🧠 MPC Controller:")
            print(f"   Avg Formation Error: {mpc_avg_error:.3f}m")
            print(f"   Avg Control Effort: {mpc_avg_effort:.3f}")
            print(f"   Avg Solve Time: {mpc_avg_solve_time:.4f}s")
        
        if self.prop_metrics['formation_errors']:
            prop_avg_error = np.mean(self.prop_metrics['formation_errors'][-100:])
            prop_avg_effort = np.mean(self.prop_metrics['control_efforts'][-100:])
            
            print(f"📐 Proportional Controller:")
            print(f"   Avg Formation Error: {prop_avg_error:.3f}m")
            print(f"   Avg Control Effort: {prop_avg_effort:.3f}")
        
        if self.mpc_metrics['formation_errors'] and self.prop_metrics['formation_errors']:
            error_improvement = (prop_avg_error - mpc_avg_error) / prop_avg_error * 100
            print(f"\n🎯 MPC Improvement: {error_improvement:.1f}% better formation accuracy")
        
        print("=" * 50)
    
    def update(self):
        """Update simulation step."""
        self.time += self.dt
        self.update_leader()
        self.update_followers()
    
    def run_demo(self):
        """Run the MPC vs Proportional control demo."""
        # Setup matplotlib
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Robot visualization
        ax1.set_xlim(-15, 15)
        ax1.set_ylim(-15, 15)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_title('🤖🚁 MPC vs Proportional Control - Multi-Robot Formation')
        
        # Performance metrics visualization
        ax2.set_title('📊 Real-time Performance Comparison')
        ax2.set_xlabel('Time Steps')
        ax2.set_ylabel('Formation Error (m)')
        ax2.grid(True, alpha=0.3)
        
        # Robot markers
        leader_marker = ax1.plot([], [], 'r*', markersize=15, label='🔴 Leader')[0]
        follower_markers = [
            ax1.plot([], [], 'bo', markersize=10, label='🔵 Follower 1')[0],
            ax1.plot([], [], 'gs', markersize=10, label='🟢 Follower 2')[0],
            ax1.plot([], [], 'mo', markersize=10, label='🟠 Follower 3')[0]
        ]
        
        # Performance lines
        mpc_line = ax2.plot([], [], 'b-', linewidth=2, label='MPC Controller')[0]
        prop_line = ax2.plot([], [], 'r-', linewidth=2, label='Proportional Controller')[0]
        
        # Trails
        leader_trail = ax1.plot([], [], 'r-', alpha=0.3, linewidth=1)[0]
        follower_trails = [
            ax1.plot([], [], 'b-', alpha=0.3, linewidth=1)[0],
            ax1.plot([], [], 'g-', alpha=0.3, linewidth=1)[0],
            ax1.plot([], [], 'm-', alpha=0.3, linewidth=1)[0]
        ]
        
        # Formation targets
        target_markers = [
            ax1.plot([], [], 'kx', markersize=8, alpha=0.5)[0] for _ in range(3)
        ]
        
        # Legends and info
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')
        
        # Info text
        info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, 
                           verticalalignment='top', fontsize=10,
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        def on_key_press(event):
            if event.key == ' ':  # Spacebar
                self.switch_formation()
            elif event.key.lower() == 'm':
                self.toggle_controller()
            elif event.key.lower() == 'p':
                self.show_performance_metrics()
        
        fig.canvas.mpl_connect('key_press_event', on_key_press)
        
        plt.tight_layout()
        
        print("🎮 Demo Controls:")
        print("   SPACEBAR: Switch formation")
        print("   M: Toggle MPC/Proportional")
        print("   P: Show metrics")
        print("   Ctrl+C: Stop")
        
        try:
            step = 0
            while True:
                # Update simulation
                self.update()
                step += 1
                
                # Update robot positions
                leader_marker.set_data([self.leader.x], [self.leader.y])
                for i, follower in enumerate(self.followers):
                    follower_markers[i].set_data([follower.x], [follower.y])
                
                # Update trails
                if len(self.leader.position_history) > 1:
                    leader_trail.set_data(*zip(*self.leader.position_history))
                
                for i, follower in enumerate(self.followers):
                    if len(follower.position_history) > 1:
                        follower_trails[i].set_data(*zip(*follower.position_history))
                
                # Update formation targets
                leader_pos = np.array([self.leader.x, self.leader.y])
                targets = self.get_formation_targets(leader_pos)
                for i, target in enumerate(targets):
                    target_markers[i].set_data([target[0]], [target[1]])
                
                # Update performance plot
                if step > 10:  # Start plotting after some data
                    time_steps = range(len(self.mpc_metrics['formation_errors']))
                    if self.mpc_metrics['formation_errors']:
                        mpc_line.set_data(time_steps, self.mpc_metrics['formation_errors'])
                    
                    time_steps_prop = range(len(self.prop_metrics['formation_errors']))
                    if self.prop_metrics['formation_errors']:
                        prop_line.set_data(time_steps_prop, self.prop_metrics['formation_errors'])
                    
                    # Auto-scale performance plot
                    if self.mpc_metrics['formation_errors'] or self.prop_metrics['formation_errors']:
                        all_errors = self.mpc_metrics['formation_errors'] + self.prop_metrics['formation_errors']
                        if all_errors:
                            ax2.set_xlim(0, max(len(self.mpc_metrics['formation_errors']), 
                                              len(self.prop_metrics['formation_errors'])))
                            ax2.set_ylim(0, max(all_errors[-100:]) * 1.1)
                
                # Update info text
                controller_name = "MPC" if self.use_mpc else "Proportional"
                formation_name = self.formations[self.current_formation_idx].upper()
                
                current_error = 0
                if self.use_mpc and self.mpc_metrics['formation_errors']:
                    current_error = self.mpc_metrics['formation_errors'][-1]
                elif not self.use_mpc and self.prop_metrics['formation_errors']:
                    current_error = self.prop_metrics['formation_errors'][-1]
                
                solve_time_text = ""
                if self.use_mpc and self.mpc_metrics['solve_times']:
                    solve_time_text = f"\nSolve Time: {self.mpc_metrics['solve_times'][-1]:.4f}s"
                
                info_text.set_text(
                    f"Controller: {controller_name}\n"
                    f"Formation: {formation_name}\n"
                    f"Formation Error: {current_error:.3f}m\n"
                    f"Time: {self.time:.1f}s{solve_time_text}"
                )
                
                plt.pause(0.01)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping MPC demo...")
            self.show_performance_metrics()

def main():
    """Main function to run MPC leader-follower demo."""
    print("🚀 Starting MPC-Based Leader-Follower System...")
    
    system = MPCLeaderFollowerSystem()
    system.run_demo()

if __name__ == "__main__":
    main() 