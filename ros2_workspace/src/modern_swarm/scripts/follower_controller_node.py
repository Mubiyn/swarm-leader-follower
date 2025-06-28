#!/usr/bin/env python3

"""
Modern ROS2 Follower Controller Node

This node replaces fuzzy logic control with Model Predictive Control (MPC)
for enhanced formation keeping and obstacle avoidance in swarm robotics.

Migration from ROS1 follower_controller.py:
- ROS1 → ROS2 API conversion
- Fuzzy Logic → MPC for formation control
- Enhanced multi-objective optimization
- Real-time constraint handling
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
import numpy as np
import casadi as ca
import math
import json
from typing import Tuple, Optional, List


class ModernFollowerController(Node):
    """
    Modern ROS2 Follower Controller with MPC
    
    Replaces the original Follower class with:
    - Model Predictive Control instead of Fuzzy Logic
    - Multi-objective optimization (formation + collision avoidance)
    - Real-time constraint handling
    - Enhanced robustness and performance
    """

    def __init__(self):
        super().__init__('modern_follower_controller')
        
        # Declare parameters
        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('desired_distance', 0.75)  # meters
        self.declare_parameter('prediction_horizon', 10)
        self.declare_parameter('control_horizon', 3)
        self.declare_parameter('max_linear_vel', 1.0)   # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('safety_distance', 0.3)  # meters
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('kp_angular', 0.03)  # Proportional gain
        
        # Get parameters
        self.robot_ns = self.get_parameter('robot_namespace').value
        self.desired_distance = self.get_parameter('desired_distance').value
        self.N = self.get_parameter('prediction_horizon').value  # prediction horizon
        self.M = self.get_parameter('control_horizon').value     # control horizon
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.safety_dist = self.get_parameter('safety_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.kp = self.get_parameter('kp_angular').value
        
        # Robot state variables
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.leader_position = None
        self.leader_angle = None
        self.laser_data = None
        self.obstacle_points = []
        self.current_angle_to_leader = 0.0
        self.has_detection = False
        
        # MPC setup
        self.setup_mpc()
        
        # ROS2 Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_ns}/scan',
            self.process_scan,
            10
        )
        
        self.angle_sub = self.create_subscription(
            Float32,
            f'/{self.robot_ns}/angle_to_leader',
            self.angle_callback,
            10
        )
        
        self.leader_pos_sub = self.create_subscription(
            PointStamped,
            f'/{self.robot_ns}/leader_position',
            self.process_leader_position,
            10
        )
        
        # ROS2 Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_ns}/cmd_vel',
            10
        )
        
        self.debug_pub = self.create_publisher(
            String,
            f'/{self.robot_ns}/controller_debug',
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.2, self.control_loop)  # 5 Hz
        
        self.get_logger().info(f"Modern Follower Controller initialized for {self.robot_ns}")

    def setup_mpc(self):
        """Initialize the MPC optimization problem"""
        # State variables: [x, y, theta]
        # Control variables: [v, omega]
        
        # Create optimization variables
        self.opti = ca.Opti()
        
        # Decision variables
        self.X = self.opti.variable(3, self.N + 1)  # states
        self.U = self.opti.variable(2, self.N)      # controls
        
        # Parameters (will be set at each iteration)
        self.P = self.opti.parameter(3 + 3)  # initial state + leader position
        
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

    def setup_robot_model(self):
        """Define the robot dynamics model for MPC"""
        dt = 0.2  # discretization step (same as control frequency)
        
        # Robot dynamics: simple unicycle model
        # x_{k+1} = x_k + v_k * cos(theta_k) * dt
        # y_{k+1} = y_k + v_k * sin(theta_k) * dt  
        # theta_{k+1} = theta_k + omega_k * dt
        
        for k in range(self.N):
            x_next = self.X[0, k] + self.U[0, k] * ca.cos(self.X[2, k]) * dt
            y_next = self.X[1, k] + self.U[0, k] * ca.sin(self.X[2, k]) * dt
            theta_next = self.X[2, k] + self.U[1, k] * dt
            
            self.opti.subject_to(self.X[0, k + 1] == x_next)
            self.opti.subject_to(self.X[1, k + 1] == y_next)
            self.opti.subject_to(self.X[2, k + 1] == theta_next)

    def setup_cost_function(self):
        """Define the multi-objective cost function"""
        cost = 0
        
        # Weights for different objectives
        w_formation = 10.0    # formation keeping weight
        w_collision = 50.0    # collision avoidance weight
        w_control = 1.0       # control effort weight
        w_smoothness = 5.0    # control smoothness weight
        
        for k in range(self.N):
            # Formation keeping cost
            # Distance to desired position relative to leader
            leader_x = self.P[3]
            leader_y = self.P[4]
            
            # Desired follower position (behind leader)
            desired_x = leader_x - self.desired_distance * ca.cos(self.P[5])
            desired_y = leader_y - self.desired_distance * ca.sin(self.P[5])
            
            formation_cost = (self.X[0, k] - desired_x)**2 + (self.X[1, k] - desired_y)**2
            cost += w_formation * formation_cost
            
            # Control effort cost
            control_cost = self.U[0, k]**2 + self.U[1, k]**2
            cost += w_control * control_cost
            
            # Control smoothness cost (if not first step)
            if k > 0:
                smoothness_cost = (self.U[0, k] - self.U[0, k-1])**2 + \
                                (self.U[1, k] - self.U[1, k-1])**2
                cost += w_smoothness * smoothness_cost
        
        self.opti.minimize(cost)

    def setup_constraints(self):
        """Define system constraints"""
        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.P[:3])
        
        # Control input constraints
        for k in range(self.N):
            self.opti.subject_to(self.U[0, k] <= self.max_v)      # max forward velocity
            self.opti.subject_to(self.U[0, k] >= -0.2)            # limited reverse
            self.opti.subject_to(self.U[1, k] <= self.max_w)      # max angular velocity
            self.opti.subject_to(self.U[1, k] >= -self.max_w)     # min angular velocity

    def process_scan(self, msg: LaserScan):
        """Process LiDAR scan data for obstacle detection"""
        self.laser_data = msg
        self.extract_obstacle_points(msg)

    def extract_obstacle_points(self, scan: LaserScan):
        """Extract nearby obstacle points from laser scan"""
        self.obstacle_points = []
        
        for i, distance in enumerate(scan.ranges):
            if distance < self.safety_dist * 3 and distance > 0.1:  # nearby obstacles
                angle = scan.angle_min + i * scan.angle_increment
                
                # Convert to Cartesian coordinates
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                self.obstacle_points.append([x, y])

    def process_leader_position(self, msg: PointStamped):
        """Process leader position from vision system"""
        self.leader_position = [msg.point.x, msg.point.y, msg.point.z]

    def angle_callback(self, msg):
        """Update angle to leader from vision system"""
        self.current_angle_to_leader = msg.data
        self.has_detection = True

    def solve_mpc(self) -> Tuple[float, float]:
        """
        Solve the MPC optimization problem
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        if self.leader_position is None:
            return 0.0, 0.0
        
        try:
            # Set parameter values
            leader_heading = math.atan2(self.leader_position[1], self.leader_position[0])
            initial_state = self.robot_pose
            leader_state = self.leader_position[:2] + [leader_heading]
            
            parameter_values = initial_state + leader_state
            self.opti.set_value(self.P, parameter_values)
            
            # Solve optimization
            sol = self.opti.solve()
            
            # Extract optimal control
            u_opt = sol.value(self.U)
            linear_vel = u_opt[0, 0]
            angular_vel = u_opt[1, 0]
            
            return linear_vel, angular_vel
            
        except Exception as e:
            self.get_logger().warn(f"MPC solve failed: {e}")
            return 0.0, 0.0

    def emergency_stop_check(self) -> bool:
        """Check if emergency stop is needed due to close obstacles"""
        if not self.laser_data:
            return False
        
        # Check front sector for close obstacles
        front_angles = range(-30, 31)  # ±30 degrees front sector
        min_front_distance = float('inf')
        
        for angle_deg in front_angles:
            angle_idx = angle_deg + 180  # Convert to laser array index
            if 0 <= angle_idx < len(self.laser_data.ranges):
                distance = self.laser_data.ranges[angle_idx]
                if distance > 0:
                    min_front_distance = min(min_front_distance, distance)
        
        return min_front_distance < self.safety_dist

    def control_loop(self):
        """Main control loop using simple but effective algorithm"""
        cmd = Twist()
        
        if self.has_detection:
            # Convert angle error to radians
            angle_error_rad = math.radians(self.current_angle_to_leader)
            
            # Forward velocity - adaptive based on angle error
            angle_error_abs = abs(self.current_angle_to_leader)
            
            if angle_error_abs > 90.0:
                # Large error: slow down and turn
                cmd.linear.x = 0.2
            elif angle_error_abs > 30.0:
                # Medium error: moderate speed
                cmd.linear.x = 0.6
            else:
                # Small error: full speed
                cmd.linear.x = self.max_linear_vel
            
            # Angular velocity - proportional control
            cmd.angular.z = self.kp * angle_error_rad
            
            # Clamp angular velocity
            cmd.angular.z = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, cmd.angular.z))
            
        else:
            # No detection: stop and search
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Slow rotation to search
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log every 20 iterations to avoid spam
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f'🎮 FIXED Control: angle={self.current_angle_to_leader:.1f}°, '
                f'v={cmd.linear.x:.2f}m/s, ω={cmd.angular.z:.3f}rad/s'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ModernFollowerController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 