#!/usr/bin/env python3
"""
Advanced Controllers for ROS2 Swarm System

This module provides advanced control algorithms for the swarm system:
- Model Predictive Control (MPC) for formation keeping
- Performance monitoring and metrics
- Enhanced obstacle avoidance
- Real-time parameter tuning

Author: Modern Swarm Leader-Follower System
Phase: Advanced Control Integration
"""

import numpy as np
import math
import time
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from collections import deque

# Try to import CasADi for MPC
try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    print("⚠️ CasADi not available. MPC will use simplified implementation.")


@dataclass
class PerformanceMetrics:
    """Performance metrics for the swarm system"""
    formation_error: float = 0.0
    control_effort: float = 0.0
    solve_time: float = 0.0
    collision_count: int = 0
    obstacle_avoidance_events: int = 0
    timestamp: float = 0.0


class MPCController:
    """Model Predictive Control for formation keeping"""
    
    def __init__(self, num_robots: int = 3, prediction_horizon: int = 10, 
                 control_horizon: int = 3, dt: float = 0.1):
        self.num_robots = num_robots
        self.N = prediction_horizon
        self.M = control_horizon
        self.dt = dt
        
        # Control limits
        self.max_linear_vel = 1.0
        self.max_angular_vel = 2.0
        
        # Cost function weights
        self.w_formation = 10.0
        self.w_collision = 50.0
        self.w_control = 1.0
        self.w_smoothness = 5.0
        
        # Performance tracking
        self.performance_history = deque(maxlen=1000)
        self.last_solve_time = 0.0
        
        # Initialize MPC if CasADi is available
        if CASADI_AVAILABLE:
            self.setup_mpc()
        else:
            self.setup_simplified_mpc()
    
    def setup_mpc(self):
        """Setup CasADi-based MPC optimization problem"""
        # State variables: [x, y, theta] for each robot
        # Control variables: [v, omega] for each robot
        
        self.opti = ca.Opti()
        
        # Decision variables
        self.X = self.opti.variable(3 * self.num_robots, self.N + 1)  # states
        self.U = self.opti.variable(2 * self.num_robots, self.N)      # controls
        
        # Parameters (initial states + formation targets)
        self.P = self.opti.parameter(3 * self.num_robots + 2 * self.num_robots)
        
        # Define robot dynamics model
        self.setup_robot_model()
        
        # Define cost function
        self.setup_cost_function()
        
        # Define constraints
        self.setup_constraints()
        
        # Solver options
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 1e-3,
            'ipopt.max_iter': 100
        }
        self.opti.solver('ipopt', opts)
    
    def setup_simplified_mpc(self):
        """Setup simplified MPC without CasADi"""
        self.simplified_mode = True
        print("📊 Using simplified MPC implementation")
    
    def setup_robot_model(self):
        """Define the robot dynamics model for MPC"""
        for k in range(self.N):
            for i in range(self.num_robots):
                # Robot state indices
                idx_x = i * 3
                idx_u = i * 2
                
                # Robot dynamics: simple unicycle model
                x_next = self.X[idx_x, k] + self.U[idx_u, k] * ca.cos(self.X[idx_x + 2, k]) * self.dt
                y_next = self.X[idx_x + 1, k] + self.U[idx_u, k] * ca.sin(self.X[idx_x + 2, k]) * self.dt
                theta_next = self.X[idx_x + 2, k] + self.U[idx_u + 1, k] * self.dt
                
                self.opti.subject_to(self.X[idx_x, k + 1] == x_next)
                self.opti.subject_to(self.X[idx_x + 1, k + 1] == y_next)
                self.opti.subject_to(self.X[idx_x + 2, k + 1] == theta_next)
    
    def setup_cost_function(self):
        """Define the multi-objective cost function"""
        cost = 0
        
        for k in range(self.N):
            # Formation keeping cost
            for i in range(self.num_robots):
                idx_x = i * 3
                idx_p = 3 * self.num_robots + i * 2
                
                # Distance to desired position
                formation_cost = (self.X[idx_x, k] - self.P[idx_p])**2 + \
                               (self.X[idx_x + 1, k] - self.P[idx_p + 1])**2
                cost += self.w_formation * formation_cost
            
            # Control effort cost
            for i in range(self.num_robots):
                idx_u = i * 2
                control_cost = self.U[idx_u, k]**2 + self.U[idx_u + 1, k]**2
                cost += self.w_control * control_cost
            
            # Control smoothness cost
            if k > 0:
                for i in range(self.num_robots):
                    idx_u = i * 2
                    smoothness_cost = (self.U[idx_u, k] - self.U[idx_u, k-1])**2 + \
                                    (self.U[idx_u + 1, k] - self.U[idx_u + 1, k-1])**2
                    cost += self.w_smoothness * smoothness_cost
        
        self.opti.minimize(cost)
    
    def setup_constraints(self):
        """Define system constraints"""
        # Initial state constraint
        for i in range(self.num_robots):
            idx_x = i * 3
            idx_p = i * 3
            self.opti.subject_to(self.X[idx_x:idx_x + 3, 0] == self.P[idx_p:idx_p + 3])
        
        # Control input constraints
        for k in range(self.N):
            for i in range(self.num_robots):
                idx_u = i * 2
                self.opti.subject_to(self.U[idx_u, k] <= self.max_linear_vel)
                self.opti.subject_to(self.U[idx_u, k] >= -0.2)
                self.opti.subject_to(self.U[idx_u + 1, k] <= self.max_angular_vel)
                self.opti.subject_to(self.U[idx_u + 1, k] >= -self.max_angular_vel)
    
    def compute_control(self, robot_states: List[np.ndarray], 
                       formation_targets: List[np.ndarray]) -> List[Tuple[float, float]]:
        """Compute MPC controls for all robots"""
        start_time = time.time()
        
        if CASADI_AVAILABLE and not hasattr(self, 'simplified_mode'):
            return self.compute_mpc_control(robot_states, formation_targets)
        else:
            return self.compute_simplified_control(robot_states, formation_targets)
    
    def compute_mpc_control(self, robot_states: List[np.ndarray], 
                           formation_targets: List[np.ndarray]) -> List[Tuple[float, float]]:
        """Compute control using CasADi MPC"""
        start_time = time.time()
        try:
            # Prepare parameter values
            param_values = []
            for state in robot_states:
                param_values.extend(state[:3])  # x, y, theta
            for target in formation_targets:
                param_values.extend(target[:2])  # x, y
            
            self.opti.set_value(self.P, param_values)
            
            # Solve optimization
            sol = self.opti.solve()
            
            # Extract optimal controls
            u_opt = sol.value(self.U)
            controls = []
            
            for i in range(self.num_robots):
                idx_u = i * 2
                linear_vel = u_opt[idx_u, 0]
                angular_vel = u_opt[idx_u + 1, 0]
                controls.append((linear_vel, angular_vel))
            
            # Record performance
            solve_time = time.time() - start_time
            self.record_performance(robot_states, formation_targets, solve_time)
            
            return controls
            
        except Exception as e:
            print(f"⚠️ MPC solve failed: {e}")
            return self.compute_simplified_control(robot_states, formation_targets)
    
    def compute_simplified_control(self, robot_states: List[np.ndarray], 
                                  formation_targets: List[np.ndarray]) -> List[Tuple[float, float]]:
        """Compute simplified control (fallback)"""
        start_time = time.time()
        controls = []
        
        for i, (state, target) in enumerate(zip(robot_states, formation_targets)):
            # Simple proportional control
            error_x = target[0] - state[0]
            error_y = target[1] - state[1]
            distance_error = math.sqrt(error_x**2 + error_y**2)
            
            # Desired heading
            desired_theta = math.atan2(error_y, error_x)
            angle_error = desired_theta - state[2]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            # Control outputs
            linear_vel = min(0.5 * distance_error, self.max_linear_vel)
            angular_vel = 2.0 * angle_error
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
            
            controls.append((linear_vel, angular_vel))
        
        # Record performance
        solve_time = time.time() - start_time
        self.record_performance(robot_states, formation_targets, solve_time)
        
        return controls
    
    def record_performance(self, robot_states: List[np.ndarray], 
                          formation_targets: List[np.ndarray], solve_time: float):
        """Record performance metrics"""
        # Calculate formation error
        total_formation_error = 0.0
        for state, target in zip(robot_states, formation_targets):
            error = math.sqrt((state[0] - target[0])**2 + (state[1] - target[1])**2)
            total_formation_error += error
        
        avg_formation_error = total_formation_error / len(robot_states)
        
        # Calculate control effort (simplified)
        control_effort = 0.0  # Would need actual control values
        
        # Create metrics
        metrics = PerformanceMetrics(
            formation_error=avg_formation_error,
            control_effort=control_effort,
            solve_time=solve_time,
            timestamp=time.time()
        )
        
        self.performance_history.append(metrics)
        self.last_solve_time = solve_time
    
    def get_performance_summary(self) -> Dict[str, float]:
        """Get summary of recent performance"""
        if len(self.performance_history) == 0:
            return {}
        
        recent_metrics = list(self.performance_history)[-100:]  # Last 100 samples
        
        return {
            'avg_formation_error': np.mean([m.formation_error for m in recent_metrics]),
            'avg_solve_time': np.mean([m.solve_time for m in recent_metrics]),
            'max_solve_time': max([m.solve_time for m in recent_metrics]),
            'total_collisions': sum([m.collision_count for m in recent_metrics]),
            'obstacle_events': sum([m.obstacle_avoidance_events for m in recent_metrics])
        }


class EnhancedObstacleAvoidance:
    """Enhanced obstacle avoidance with prediction and classification"""
    
    def __init__(self, detection_range: float = 5.0, safety_distance: float = 1.5):
        self.detection_range = detection_range
        self.safety_distance = safety_distance
        self.obstacle_history = {}  # Track obstacle trajectories
        self.avoidance_events = 0
    
    def predict_obstacle_position(self, obstacle, prediction_time: float = 1.0) -> Tuple[float, float]:
        """Predict obstacle position in the future"""
        if hasattr(obstacle, 'vx') and hasattr(obstacle, 'vy'):
            # Dynamic obstacle
            predicted_x = obstacle.x + obstacle.vx * prediction_time
            predicted_y = obstacle.y + obstacle.vy * prediction_time
            return predicted_x, predicted_y
        else:
            # Static obstacle
            return obstacle.x, obstacle.y
    
    def calculate_enhanced_avoidance(self, robot, obstacles: List) -> Tuple[float, float]:
        """Calculate enhanced avoidance forces with prediction"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for obstacle in obstacles:
            # Predict obstacle position
            pred_x, pred_y = self.predict_obstacle_position(obstacle)
            
            # Calculate distance to predicted position
            dx = robot.x - pred_x
            dy = robot.y - pred_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.detection_range:
                required_distance = obstacle.radius + self.safety_distance
                
                if distance < required_distance:
                    # Enhanced repulsive force
                    force_magnitude = 3.0 * (required_distance - distance) / max(distance, 0.1)
                    avoidance_x += force_magnitude * dx
                    avoidance_y += force_magnitude * dy
                    
                    self.avoidance_events += 1
        
        # Limit avoidance force
        max_avoidance = 2.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y


class PerformanceMonitor:
    """Real-time performance monitoring and visualization"""
    
    def __init__(self):
        self.metrics_history = deque(maxlen=1000)
        self.formation_errors = deque(maxlen=1000)
        self.control_efforts = deque(maxlen=1000)
        self.solve_times = deque(maxlen=1000)
        self.start_time = time.time()
    
    def update_metrics(self, formation_error: float, control_effort: float, 
                      solve_time: float, collision_count: int = 0):
        """Update performance metrics"""
        timestamp = time.time() - self.start_time
        
        self.formation_errors.append(formation_error)
        self.control_efforts.append(control_effort)
        self.solve_times.append(solve_time)
        
        metrics = PerformanceMetrics(
            formation_error=formation_error,
            control_effort=control_effort,
            solve_time=solve_time,
            collision_count=collision_count,
            timestamp=timestamp
        )
        
        self.metrics_history.append(metrics)
    
    def get_recent_performance(self, window: int = 100) -> Dict[str, float]:
        """Get performance summary for recent window"""
        if len(self.metrics_history) == 0:
            return {}
        
        recent = list(self.metrics_history)[-window:]
        
        return {
            'avg_formation_error': np.mean([m.formation_error for m in recent]),
            'avg_control_effort': np.mean([m.control_effort for m in recent]),
            'avg_solve_time': np.mean([m.solve_time for m in recent]),
            'max_solve_time': max([m.solve_time for m in recent]),
            'total_collisions': sum([m.collision_count for m in recent]),
            'runtime': time.time() - self.start_time
        }
    
    def get_performance_trends(self) -> Dict[str, List[float]]:
        """Get performance trends for visualization"""
        return {
            'formation_errors': list(self.formation_errors),
            'control_efforts': list(self.control_efforts),
            'solve_times': list(self.solve_times)
        } 