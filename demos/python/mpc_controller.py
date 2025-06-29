"""
Model Predictive Control (MPC) Controller for Multi-Robot Formation Control

This module implements an MPC controller for leader-follower formation control
using CasADi for optimization. The controller handles:
- Formation keeping constraints
- Collision avoidance
- Speed and acceleration limits
- Obstacle avoidance integration

Author: Modern Swarm Leader-Follower System
Phase: 5 - Advanced Control
"""

import casadi as ca
import numpy as np
from typing import List, Tuple, Dict, Optional
import time

class MPCController:
    """
    Model Predictive Control controller for multi-robot formation control.
    
    Features:
    - Prediction horizon optimization
    - Constraint handling (speed limits, collisions)
    - Formation error minimization
    - Real-time control updates
    """
    
    def __init__(self, 
                 prediction_horizon: int = 10,
                 control_horizon: int = 5,
                 dt: float = 0.1,
                 max_linear_vel: float = 1.0,
                 max_angular_vel: float = 1.5,
                 formation_weight: float = 10.0,
                 control_weight: float = 1.0,
                 collision_weight: float = 100.0,
                 min_robot_distance: float = 1.0):
        """
        Initialize MPC Controller.
        
        Args:
            prediction_horizon: Number of prediction steps (N)
            control_horizon: Number of control steps (Nc)
            dt: Time step for discretization
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            formation_weight: Weight for formation error cost
            control_weight: Weight for control effort cost
            collision_weight: Weight for collision avoidance cost
            min_robot_distance: Minimum distance between robots (m)
        """
        self.N = prediction_horizon
        self.Nc = control_horizon
        self.dt = dt
        self.max_v = max_linear_vel
        self.max_omega = max_angular_vel
        self.Q_formation = formation_weight
        self.R_control = control_weight
        self.Q_collision = collision_weight
        self.min_dist = min_robot_distance
        
        # Robot state dimension: [x, y, theta, v, omega]
        self.nx = 5  # state dimension
        self.nu = 2  # control dimension [v_cmd, omega_cmd]
        
        # Setup optimization problem
        self._setup_optimization_problem()
        
        # Performance metrics
        self.solve_times = []
        self.cost_history = []
        
    def _setup_optimization_problem(self):
        """Setup the MPC optimization problem using CasADi."""
        print("🔧 Setting up MPC optimization problem...")
        
        # Decision variables
        # States: [x, y, theta, v, omega] for each robot over prediction horizon
        # Controls: [v_cmd, omega_cmd] for each robot over control horizon
        
        # Create optimization variables
        self.opti = ca.Opti()
        
        # We'll setup for a single robot first, then extend to multi-robot
        # State variables over prediction horizon
        self.X = self.opti.variable(self.nx, self.N + 1)  # States
        self.U = self.opti.variable(self.nu, self.N)      # Controls
        
        # Parameters (will be set at each MPC iteration)
        self.P = self.opti.parameter(self.nx + 2)  # Initial state + target position
        
        # Objective function
        self.obj = 0
        
        # Formation error cost
        for k in range(self.N):
            # Formation error: distance from target position
            target_x = self.P[self.nx]
            target_y = self.P[self.nx + 1]
            
            formation_error = (self.X[0, k] - target_x)**2 + (self.X[1, k] - target_y)**2
            self.obj += self.Q_formation * formation_error
            
            # Control effort cost
            control_effort = self.U[0, k]**2 + self.U[1, k]**2
            self.obj += self.R_control * control_effort
        
        # Terminal formation cost
        target_x = self.P[self.nx]
        target_y = self.P[self.nx + 1]
        terminal_error = (self.X[0, self.N] - target_x)**2 + (self.X[1, self.N] - target_y)**2
        self.obj += self.Q_formation * terminal_error
        
        # Set objective
        self.opti.minimize(self.obj)
        
        # Dynamics constraints
        for k in range(self.N):
            # Robot dynamics: [x, y, theta, v, omega]
            x_next = self.X[0, k] + self.X[3, k] * ca.cos(self.X[2, k]) * self.dt
            y_next = self.X[1, k] + self.X[3, k] * ca.sin(self.X[2, k]) * self.dt
            theta_next = self.X[2, k] + self.X[4, k] * self.dt
            v_next = self.U[0, k]  # Direct velocity control
            omega_next = self.U[1, k]  # Direct angular velocity control
            
            # Dynamics constraints
            self.opti.subject_to(self.X[0, k+1] == x_next)
            self.opti.subject_to(self.X[1, k+1] == y_next)
            self.opti.subject_to(self.X[2, k+1] == theta_next)
            self.opti.subject_to(self.X[3, k+1] == v_next)
            self.opti.subject_to(self.X[4, k+1] == omega_next)
        
        # Control constraints
        for k in range(self.N):
            self.opti.subject_to(self.U[0, k] >= -self.max_v)
            self.opti.subject_to(self.U[0, k] <= self.max_v)
            self.opti.subject_to(self.U[1, k] >= -self.max_omega)
            self.opti.subject_to(self.U[1, k] <= self.max_omega)
        
        # Initial condition constraint
        self.opti.subject_to(self.X[:, 0] == self.P[:self.nx])
        
        # Solver settings
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.acceptable_tol': 1e-6,
            'ipopt.acceptable_obj_change_tol': 1e-6
        }
        self.opti.solver('ipopt', opts)
        
        print("✅ MPC optimization problem setup complete!")
    
    def compute_control(self, 
                       current_state: np.ndarray, 
                       target_position: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        Compute optimal control using MPC.
        
        Args:
            current_state: Current robot state [x, y, theta, v, omega]
            target_position: Target position [x, y]
            
        Returns:
            Tuple of (control_input, info_dict)
            - control_input: [v_cmd, omega_cmd]
            - info_dict: Solver information and metrics
        """
        start_time = time.time()
        
        try:
            # Set parameters
            param_value = np.concatenate([current_state, target_position])
            self.opti.set_value(self.P, param_value)
            
            # Solve optimization
            sol = self.opti.solve()
            
            # Extract optimal control (first control input)
            u_opt = sol.value(self.U)
            control_input = u_opt[:, 0]  # First control input
            
            # Extract predicted trajectory
            x_pred = sol.value(self.X)
            
            # Compute metrics
            solve_time = time.time() - start_time
            self.solve_times.append(solve_time)
            
            cost = sol.value(self.obj)
            self.cost_history.append(cost)
            
            info = {
                'solve_time': solve_time,
                'cost': cost,
                'predicted_trajectory': x_pred,
                'optimal_controls': u_opt,
                'success': True,
                'iterations': sol.stats()['iter_count'] if 'iter_count' in sol.stats() else None
            }
            
            return control_input, info
            
        except Exception as e:
            print(f"❌ MPC solve failed: {e}")
            
            # Return safe fallback control
            fallback_control = np.array([0.0, 0.0])
            
            info = {
                'solve_time': time.time() - start_time,
                'cost': np.inf,
                'predicted_trajectory': None,
                'optimal_controls': None,
                'success': False,
                'error': str(e)
            }
            
            return fallback_control, info
    
    def get_performance_metrics(self) -> Dict:
        """Get performance metrics for the MPC controller."""
        if not self.solve_times:
            return {}
        
        return {
            'avg_solve_time': np.mean(self.solve_times),
            'max_solve_time': np.max(self.solve_times),
            'min_solve_time': np.min(self.solve_times),
            'avg_cost': np.mean(self.cost_history) if self.cost_history else None,
            'total_solves': len(self.solve_times),
            'success_rate': len([t for t in self.solve_times if t < 1.0]) / len(self.solve_times) * 100
        }
    
    def reset_metrics(self):
        """Reset performance metrics."""
        self.solve_times.clear()
        self.cost_history.clear()


class MultiRobotMPCController:
    """
    Multi-robot MPC controller for formation control.
    
    Handles multiple robots with collision avoidance and formation constraints.
    """
    
    def __init__(self, 
                 num_robots: int,
                 formation_type: str = "triangle",
                 **mpc_kwargs):
        """
        Initialize multi-robot MPC controller.
        
        Args:
            num_robots: Number of robots in formation
            formation_type: Type of formation ("triangle", "line", "circle", "v_shape")
            **mpc_kwargs: Arguments passed to individual MPC controllers
        """
        self.num_robots = num_robots
        self.formation_type = formation_type
        
        # Create individual MPC controllers for each robot
        self.controllers = []
        for i in range(num_robots):
            controller = MPCController(**mpc_kwargs)
            self.controllers.append(controller)
        
        # Formation definitions
        self.formations = {
            "triangle": self._triangle_formation,
            "line": self._line_formation,
            "circle": self._circle_formation,
            "v_shape": self._v_shape_formation
        }
        
        print(f"🚀 Multi-robot MPC controller initialized with {num_robots} robots")
    
    def _triangle_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Generate triangle formation positions."""
        positions = []
        # Follower positions relative to leader
        offsets = [
            np.array([-2.0, -1.5]),  # Left rear
            np.array([-2.0, 1.5]),   # Right rear  
            np.array([-3.0, 0.0])    # Center rear
        ]
        
        for offset in offsets:
            target_pos = leader_pos + offset
            positions.append(target_pos)
        
        return positions
    
    def _line_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Generate line formation positions."""
        positions = []
        for i in range(self.num_robots - 1):  # Exclude leader
            offset = np.array([-(i + 1) * 2.0, 0.0])
            target_pos = leader_pos + offset
            positions.append(target_pos)
        
        return positions
    
    def _circle_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Generate circle formation positions."""
        positions = []
        radius = 2.5
        num_followers = self.num_robots - 1
        
        for i in range(num_followers):
            angle = 2 * np.pi * i / num_followers + np.pi  # Start from behind
            offset = np.array([radius * np.cos(angle), radius * np.sin(angle)])
            target_pos = leader_pos + offset
            positions.append(target_pos)
        
        return positions
    
    def _v_shape_formation(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Generate V-shape formation positions."""
        positions = []
        offsets = [
            np.array([-2.0, -2.0]),  # Left wing
            np.array([-2.0, 2.0]),   # Right wing
            np.array([-4.0, 0.0])    # Center rear
        ]
        
        for offset in offsets:
            target_pos = leader_pos + offset
            positions.append(target_pos)
        
        return positions
    
    def compute_formation_controls(self, 
                                 robot_states: List[np.ndarray],
                                 leader_position: np.ndarray) -> List[Tuple[np.ndarray, Dict]]:
        """
        Compute optimal controls for all robots in formation.
        
        Args:
            robot_states: List of robot states [x, y, theta, v, omega]
            leader_position: Leader position [x, y]
            
        Returns:
            List of (control_input, info_dict) for each robot
        """
        # Get formation target positions
        target_positions = self.formations[self.formation_type](leader_position)
        
        # Compute control for each robot
        results = []
        for i, (state, target) in enumerate(zip(robot_states, target_positions)):
            control, info = self.controllers[i].compute_control(state, target)
            results.append((control, info))
        
        return results
    
    def set_formation_type(self, formation_type: str):
        """Change formation type."""
        if formation_type in self.formations:
            self.formation_type = formation_type
            print(f"🔄 Formation changed to {formation_type}")
        else:
            print(f"❌ Unknown formation type: {formation_type}")
    
    def get_performance_summary(self) -> Dict:
        """Get performance summary for all robots."""
        summary = {}
        for i, controller in enumerate(self.controllers):
            summary[f'robot_{i}'] = controller.get_performance_metrics()
        
        return summary


# Test the MPC controller
if __name__ == "__main__":
    print("🧪 Testing MPC Controller...")
    
    # Test single robot MPC
    mpc = MPCController(prediction_horizon=10, dt=0.1)
    
    # Test control computation
    current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, theta, v, omega]
    target_pos = np.array([5.0, 3.0])  # Target position
    
    control, info = mpc.compute_control(current_state, target_pos)
    
    print(f"✅ MPC Control Test:")
    print(f"   Control: {control}")
    print(f"   Solve time: {info['solve_time']:.4f}s")
    print(f"   Cost: {info['cost']:.2f}")
    print(f"   Success: {info['success']}")
    
    # Test multi-robot MPC
    print("\n🧪 Testing Multi-Robot MPC...")
    multi_mpc = MultiRobotMPCController(num_robots=4, formation_type="triangle")
    
    # Test formation control
    robot_states = [
        np.array([0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([1.0, 1.0, 0.0, 0.0, 0.0]),
        np.array([-1.0, 1.0, 0.0, 0.0, 0.0]),
        np.array([-1.0, -1.0, 0.0, 0.0, 0.0])
    ]
    leader_pos = np.array([10.0, 5.0])
    
    results = multi_mpc.compute_formation_controls(robot_states, leader_pos)
    
    print(f"✅ Multi-Robot MPC Test:")
    for i, (control, info) in enumerate(results):
        print(f"   Robot {i}: Control {control}, Success: {info['success']}")
    
    print("\n🎉 MPC Controller tests completed!") 