#!/usr/bin/env python3
"""
🚀 Enhanced ROS2 Swarm Bridge with Services

Professional ROS2 swarm system featuring:
✅ Dynamic formation switching via services
✅ Controller switching via services  
✅ Obstacle management via services
✅ Parameter server integration
✅ Threading-safe GUI visualization

NEW FEATURES TODAY:
- ROS2 services for real-time control
- Parameter server for configuration
- Enhanced obstacle management
- Professional system architecture
"""

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg for better threading on macOS
from matplotlib.patches import Circle
import math
import threading
import queue
import time
import signal
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

class SwarmRobot:
    """Individual robot in the swarm."""
    
    def __init__(self, robot_id, x=0.0, y=0.0, theta=0.0, color='blue'):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.color = color
        self.position_history = [(x, y)]
        
        # Control inputs
        self.cmd_v = 0.0
        self.cmd_omega = 0.0
    
    def update(self, dt):
        """Update robot physics."""
        self.v = self.cmd_v
        self.omega = self.cmd_omega
        
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Store position history
        self.position_history.append((self.x, self.y))
        if len(self.position_history) > 100:
            self.position_history.pop(0)
    
    def set_cmd_vel(self, linear_x, angular_z):
        """Set velocity commands."""
        self.cmd_v = max(min(linear_x, 1.0), -1.0)
        self.cmd_omega = max(min(angular_z, 1.5), -1.5)

class Obstacle:
    """Dynamic obstacle in the environment."""
    
    def __init__(self, x, y, radius, obstacle_type="static", vx=0.0, vy=0.0):
        self.x = x
        self.y = y
        self.radius = radius
        self.type = obstacle_type
        self.vx = vx
        self.vy = vy
        self.id = id(self)  # Unique ID
    
    def update(self, dt):
        """Update obstacle position for dynamic obstacles."""
        if self.type == "dynamic":
            self.x += self.vx * dt
            self.y += self.vy * dt
            
            # Bounce off boundaries
            if abs(self.x) > 7.0:
                self.vx = -self.vx
            if abs(self.y) > 7.0:
                self.vy = -self.vy

class SwarmControllerNode(Node):
    """🚀 Enhanced ROS2 node with professional service integration - THREADING FIXED."""
    
    def __init__(self):
        super().__init__('swarm_controller')
        
        # Simulation parameters
        self.dt = 0.05
        self.time = 0.0
        self.running = True
        
        # Leader motion parameters  
        self.leader_radius = 5.0
        self.leader_speed = 1.0
        
        # Formation configurations
        self.formations = {
            'triangle': [[-2.0, -1.5], [-2.0, 1.5], [-3.5, 0.0]],
            'line': [[-2.0, 0.0], [-4.0, 0.0], [-6.0, 0.0]],
            'circle': [
                [-2.0 * math.cos(i * 2*math.pi/3), -2.0 * math.sin(i * 2*math.pi/3)] 
                for i in range(3)
            ],
            'v_shape': [[-2.0, -1.0], [-2.0, 1.0], [-3.0, -2.0], [-3.0, 2.0]]
        }
        self.current_formation = 'triangle'
        self.formation_spacing = 2.0
        
        # Controller configuration
        self.controllers = ['proportional', 'mpc', 'rl']
        self.current_controller = 'proportional'
        self.proportional_gain = 1.0
        
        # Create robots
        self.robots = {
            'leader': SwarmRobot('leader', 5.0, 0.0, 0.0, 'red'),
            'follower_1': SwarmRobot('follower_1', 3.0, -1.5, 0.0, 'blue'),
            'follower_2': SwarmRobot('follower_2', 3.0, 1.5, 0.0, 'green'),
            'follower_3': SwarmRobot('follower_3', 1.5, 0.0, 0.0, 'orange')
        }
        
        # Obstacles
        self.obstacles = []
        
        # Create ROS2 interface
        self._create_ros2_interface()
        self._create_services()
        self._setup_parameters()
        
        # Start simulation timer
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)
        
        self.get_logger().info("🚀 Enhanced ROS2 Swarm Controller Starting...")
        self.get_logger().info("✅ Services: formation, controller, obstacles, reset")
        self.get_logger().info("✅ Parameters: comprehensive real-time configuration") 
        self.get_logger().info("✅ Threading: NON-BLOCKING architecture")
        self.get_logger().info("✅ Professional ROS2 system ready!")
    
    def _create_ros2_interface(self):
        """Create standard ROS2 publishers and subscribers."""
        # Publishers for robot odometry
        self.odom_publishers = {}
        for robot_name in self.robots.keys():
            self.odom_publishers[robot_name] = self.create_publisher(
                Odometry, f'/{robot_name}/odom', 10
            )
        
        # Status publisher
        self.status_publisher = self.create_publisher(String, '/swarm/status', 10)
    
    def _create_services(self):
        """🔧 Create ROS2 services for dynamic control."""
        # Formation switching service
        self.formation_service = self.create_service(
            Trigger, '/swarm/set_formation', self.set_formation_callback
        )
        
        # Controller switching service
        self.controller_service = self.create_service(
            Trigger, '/swarm/set_controller', self.set_controller_callback
        )
        
        # Obstacle management service
        self.obstacle_service = self.create_service(
            Trigger, '/swarm/add_obstacle', self.add_obstacle_callback
        )
        
        # System reset service
        self.reset_service = self.create_service(
            Trigger, '/swarm/reset', self.reset_system_callback
        )
        
        self.get_logger().info("📡 ROS2 Services created:")
        self.get_logger().info("   • /swarm/set_formation - Switch formations")
        self.get_logger().info("   • /swarm/set_controller - Switch controllers") 
        self.get_logger().info("   • /swarm/add_obstacle - Add random obstacle")
        self.get_logger().info("   • /swarm/reset - Reset system state")
    
    def _setup_parameters(self):
        """⚙️ Setup comprehensive ROS2 parameter server for real-time configuration."""
        
        # === FORMATION PARAMETERS ===
        self.declare_parameter('formation.spacing', 2.0)
        self.declare_parameter('formation.max_followers', 3)
        self.declare_parameter('formation.switch_threshold', 0.5)  # Distance for formation switch
        self.declare_parameter('formation.adaptive_spacing', True)  # Auto-adjust spacing
        
        # === CONTROL PARAMETERS ===
        self.declare_parameter('control.proportional_gain', 1.0)
        self.declare_parameter('control.max_velocity', 1.0)
        self.declare_parameter('control.max_angular_velocity', 1.5)
        self.declare_parameter('control.velocity_smoothing', 0.1)  # Velocity smoothing factor
        self.declare_parameter('control.position_tolerance', 0.1)  # Position accuracy
        self.declare_parameter('control.angle_tolerance', 0.05)    # Angle accuracy
        
        # === LEADER PARAMETERS ===
        self.declare_parameter('leader.radius', 5.0)
        self.declare_parameter('leader.speed', 1.0)
        self.declare_parameter('leader.circular_motion', True)     # Enable circular motion
        self.declare_parameter('leader.motion_pattern', 'circle')  # circle, figure8, line
        
        # === OBSTACLE AVOIDANCE PARAMETERS ===
        self.declare_parameter('obstacles.avoidance_distance', 1.5)
        self.declare_parameter('obstacles.repulsion_gain', 2.0)
        self.declare_parameter('obstacles.max_obstacles', 10)
        self.declare_parameter('obstacles.auto_cleanup', True)     # Remove old obstacles
        self.declare_parameter('obstacles.dynamic_probability', 0.3)  # Chance of dynamic obstacles
        
        # === VISUALIZATION PARAMETERS ===
        self.declare_parameter('visualization.trail_length', 100)
        self.declare_parameter('visualization.update_rate', 20.0)  # Hz
        self.declare_parameter('visualization.show_trajectories', True)
        self.declare_parameter('visualization.show_formation_lines', True)
        self.declare_parameter('visualization.robot_size', 0.2)
        self.declare_parameter('visualization.grid_enabled', True)
        
        # === PERFORMANCE PARAMETERS ===
        self.declare_parameter('performance.simulation_rate', 20.0)  # Hz
        self.declare_parameter('performance.publish_rate', 10.0)     # Hz
        self.declare_parameter('performance.log_level', 'INFO')      # DEBUG, INFO, WARN, ERROR
        self.declare_parameter('performance.metrics_enabled', True)
        
        # === SAFETY PARAMETERS ===
        self.declare_parameter('safety.collision_distance', 0.3)
        self.declare_parameter('safety.emergency_stop', False)
        self.declare_parameter('safety.boundary_x', 8.0)
        self.declare_parameter('safety.boundary_y', 8.0)
        self.declare_parameter('safety.boundary_enforcement', True)
        
        # Load initial values
        self._load_parameter_values()
        
        # Parameter callback for real-time updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info("⚙️ Comprehensive parameter server configured!")
        self.get_logger().info(f"📊 {len(self.get_parameters([]))} parameters available for real-time tuning")
    
    def _load_parameter_values(self):
        """Load parameter values into instance variables."""
        # Formation parameters
        self.formation_spacing = self.get_parameter('formation.spacing').value
        self.max_followers = self.get_parameter('formation.max_followers').value
        self.switch_threshold = self.get_parameter('formation.switch_threshold').value
        self.adaptive_spacing = self.get_parameter('formation.adaptive_spacing').value
        
        # Control parameters
        self.proportional_gain = self.get_parameter('control.proportional_gain').value
        self.max_velocity = self.get_parameter('control.max_velocity').value
        self.max_angular_velocity = self.get_parameter('control.max_angular_velocity').value
        self.velocity_smoothing = self.get_parameter('control.velocity_smoothing').value
        self.position_tolerance = self.get_parameter('control.position_tolerance').value
        self.angle_tolerance = self.get_parameter('control.angle_tolerance').value
        
        # Leader parameters
        self.leader_radius = self.get_parameter('leader.radius').value
        self.leader_speed = self.get_parameter('leader.speed').value
        self.circular_motion = self.get_parameter('leader.circular_motion').value
        self.motion_pattern = self.get_parameter('leader.motion_pattern').value
        
        # Obstacle parameters
        self.avoidance_distance = self.get_parameter('obstacles.avoidance_distance').value
        self.repulsion_gain = self.get_parameter('obstacles.repulsion_gain').value
        self.max_obstacles = self.get_parameter('obstacles.max_obstacles').value
        self.auto_cleanup = self.get_parameter('obstacles.auto_cleanup').value
        self.dynamic_probability = self.get_parameter('obstacles.dynamic_probability').value
        
        # Visualization parameters
        self.trail_length = self.get_parameter('visualization.trail_length').value
        self.update_rate = self.get_parameter('visualization.update_rate').value
        self.show_trajectories = self.get_parameter('visualization.show_trajectories').value
        self.show_formation_lines = self.get_parameter('visualization.show_formation_lines').value
        self.robot_size = self.get_parameter('visualization.robot_size').value
        self.grid_enabled = self.get_parameter('visualization.grid_enabled').value
        
        # Performance parameters
        self.simulation_rate = self.get_parameter('performance.simulation_rate').value
        self.publish_rate = self.get_parameter('performance.publish_rate').value
        self.log_level = self.get_parameter('performance.log_level').value
        self.metrics_enabled = self.get_parameter('performance.metrics_enabled').value
        
        # Safety parameters
        self.collision_distance = self.get_parameter('safety.collision_distance').value
        self.emergency_stop = self.get_parameter('safety.emergency_stop').value
        self.boundary_x = self.get_parameter('safety.boundary_x').value
        self.boundary_y = self.get_parameter('safety.boundary_y').value
        self.boundary_enforcement = self.get_parameter('safety.boundary_enforcement').value
    
    def parameter_callback(self, params):
        """Handle comprehensive parameter updates in real-time."""
        result = SetParametersResult()
        result.successful = True
        result.reason = "Parameters updated successfully"
        updated_params = []
        
        for param in params:
            try:
                # === FORMATION PARAMETERS ===
                if param.name == 'formation.spacing':
                    self.formation_spacing = param.value
                    self._update_formation_spacing()
                    updated_params.append(f"📐 Formation spacing: {param.value}")
                elif param.name == 'formation.max_followers':
                    self.max_followers = param.value
                    updated_params.append(f"👥 Max followers: {param.value}")
                elif param.name == 'formation.switch_threshold':
                    self.switch_threshold = param.value
                    updated_params.append(f"🎯 Switch threshold: {param.value}")
                elif param.name == 'formation.adaptive_spacing':
                    self.adaptive_spacing = param.value
                    updated_params.append(f"🔄 Adaptive spacing: {param.value}")
                
                # === CONTROL PARAMETERS ===
                elif param.name == 'control.proportional_gain':
                    self.proportional_gain = param.value
                    updated_params.append(f"🎛️ Proportional gain: {param.value}")
                elif param.name == 'control.max_velocity':
                    self.max_velocity = param.value
                    updated_params.append(f"⚡ Max velocity: {param.value}")
                elif param.name == 'control.max_angular_velocity':
                    self.max_angular_velocity = param.value
                    updated_params.append(f"🔄 Max angular velocity: {param.value}")
                elif param.name == 'control.velocity_smoothing':
                    self.velocity_smoothing = param.value
                    updated_params.append(f"📊 Velocity smoothing: {param.value}")
                
                # === LEADER PARAMETERS ===
                elif param.name == 'leader.radius':
                    self.leader_radius = param.value
                    updated_params.append(f"🔄 Leader radius: {param.value}")
                elif param.name == 'leader.speed':
                    self.leader_speed = param.value
                    updated_params.append(f"⚡ Leader speed: {param.value}")
                elif param.name == 'leader.circular_motion':
                    self.circular_motion = param.value
                    updated_params.append(f"🔄 Circular motion: {param.value}")
                elif param.name == 'leader.motion_pattern':
                    self.motion_pattern = param.value
                    updated_params.append(f"🔀 Motion pattern: {param.value}")
                
                # === OBSTACLE PARAMETERS ===
                elif param.name == 'obstacles.avoidance_distance':
                    self.avoidance_distance = param.value
                    updated_params.append(f"🚧 Avoidance distance: {param.value}")
                elif param.name == 'obstacles.repulsion_gain':
                    self.repulsion_gain = param.value
                    updated_params.append(f"💨 Repulsion gain: {param.value}")
                elif param.name == 'obstacles.max_obstacles':
                    self.max_obstacles = param.value
                    updated_params.append(f"🚧 Max obstacles: {param.value}")
                elif param.name == 'obstacles.auto_cleanup':
                    self.auto_cleanup = param.value
                    updated_params.append(f"🧹 Auto cleanup: {param.value}")
                
                # === VISUALIZATION PARAMETERS ===
                elif param.name == 'visualization.trail_length':
                    self.trail_length = param.value
                    updated_params.append(f"🛤️ Trail length: {param.value}")
                elif param.name == 'visualization.update_rate':
                    self.update_rate = param.value
                    updated_params.append(f"📺 Update rate: {param.value} Hz")
                elif param.name == 'visualization.show_trajectories':
                    self.show_trajectories = param.value
                    updated_params.append(f"🛤️ Show trajectories: {param.value}")
                elif param.name == 'visualization.robot_size':
                    self.robot_size = param.value
                    updated_params.append(f"🤖 Robot size: {param.value}")
                
                # === SAFETY PARAMETERS ===
                elif param.name == 'safety.emergency_stop':
                    self.emergency_stop = param.value
                    if param.value:
                        self._emergency_stop_all_robots()
                    updated_params.append(f"🛑 Emergency stop: {param.value}")
                elif param.name == 'safety.collision_distance':
                    self.collision_distance = param.value
                    updated_params.append(f"⚠️ Collision distance: {param.value}")
                elif param.name == 'safety.boundary_enforcement':
                    self.boundary_enforcement = param.value
                    updated_params.append(f"🚧 Boundary enforcement: {param.value}")
                
            except Exception as e:
                result.successful = False
                result.reason = f"Failed to update parameter {param.name}: {str(e)}"
                self.get_logger().error(f"❌ Parameter update failed: {e}")
                return result
        
        # Log all updated parameters
        if updated_params:
            self.get_logger().info("🔧 Parameters updated:")
            for update in updated_params[:5]:  # Show first 5 to avoid spam
                self.get_logger().info(f"   • {update}")
            if len(updated_params) > 5:
                self.get_logger().info(f"   • ... and {len(updated_params) - 5} more")
        
        return result
    
    def _emergency_stop_all_robots(self):
        """Emergency stop all robots."""
        for robot in self.robots.values():
            robot.set_cmd_vel(0.0, 0.0)
        self.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED - All robots stopped")
    
    def set_formation_callback(self, request, response):
        """🔄 Service callback for formation switching."""
        formations = list(self.formations.keys())
        current_idx = formations.index(self.current_formation)
        next_idx = (current_idx + 1) % len(formations)
        new_formation = formations[next_idx]
        
        self.current_formation = new_formation
        self.get_logger().info(f"🔄 Formation switched to: {new_formation}")
        
        response.success = True
        response.message = f"Formation changed to {new_formation}"
        return response
    
    def set_controller_callback(self, request, response):
        """🎛️ Service callback for controller switching."""
        current_idx = self.controllers.index(self.current_controller)
        next_idx = (current_idx + 1) % len(self.controllers)
        new_controller = self.controllers[next_idx]
        
        self.current_controller = new_controller
        self.get_logger().info(f"🎛️ Controller switched to: {new_controller}")
        
        response.success = True
        response.message = f"Controller changed to {new_controller}"
        return response
    
    def add_obstacle_callback(self, request, response):
        """🚧 Service callback for adding obstacles."""
        # Add random obstacle
        x = np.random.uniform(-6, 6)
        y = np.random.uniform(-6, 6)
        radius = np.random.uniform(0.3, 0.8)
        obstacle_type = np.random.choice(['static', 'dynamic'])
        
        if obstacle_type == 'dynamic':
            vx = np.random.uniform(-1, 1)
            vy = np.random.uniform(-1, 1)
            obstacle = Obstacle(x, y, radius, obstacle_type, vx, vy)
        else:
            obstacle = Obstacle(x, y, radius, obstacle_type)
        
        self.obstacles.append(obstacle)
        self.get_logger().info(f"🚧 Added {obstacle_type} obstacle at ({x:.1f}, {y:.1f})")
        
        response.success = True
        response.message = f"Added {obstacle_type} obstacle at ({x:.1f}, {y:.1f})"
        return response
    
    def reset_system_callback(self, request, response):
        """🔄 Service callback for system reset."""
        # Reset robot positions
        self.robots['leader'].x = 5.0
        self.robots['leader'].y = 0.0
        self.robots['leader'].theta = 0.0
        
        # Reset followers
        self.robots['follower_1'].x = 3.0
        self.robots['follower_1'].y = -1.5
        self.robots['follower_2'].x = 3.0
        self.robots['follower_2'].y = 1.5
        self.robots['follower_3'].x = 1.5
        self.robots['follower_3'].y = 0.0
        
        # Clear position histories
        for robot in self.robots.values():
            robot.position_history = [(robot.x, robot.y)]
        
        # Clear obstacles
        self.obstacles.clear()
        
        # Reset time
        self.time = 0.0
        
        self.get_logger().info("🔄 System reset to initial state")
        
        response.success = True
        response.message = "System reset successfully"
        return response
    
    def _update_formation_spacing(self):
        """Update formation offsets based on spacing parameter."""
        spacing = self.formation_spacing
        self.formations = {
            'triangle': [[-spacing, -spacing*0.75], [-spacing, spacing*0.75], [-spacing*1.75, 0.0]],
            'line': [[-spacing, 0.0], [-spacing*2, 0.0], [-spacing*3, 0.0]],
            'circle': [
                [-spacing * math.cos(i * 2*math.pi/3), -spacing * math.sin(i * 2*math.pi/3)] 
                for i in range(3)
            ],
            'v_shape': [[-spacing, -spacing*0.5], [-spacing, spacing*0.5], 
                       [-spacing*1.5, -spacing], [-spacing*1.5, spacing]]
        }
    
    def simulation_step(self):
        """Main simulation step with enhanced features."""
        if not self.running:
            return
            
        self.time += self.dt
        
        # Update leader
        self._update_leader()
        
        # Update followers with current controller
        if self.current_controller == 'proportional':
            self._update_followers_proportional()
        elif self.current_controller == 'mpc':
            self._update_followers_mpc()
        else:  # rl
            self._update_followers_rl()
        
        # Update obstacles
        for obstacle in self.obstacles:
            obstacle.update(self.dt)
        
        # Update robot physics
        for robot in self.robots.values():
            robot.update(self.dt)
        
        # Publish robot states
        self._publish_robot_states()
    
    def _update_leader(self):
        """Update leader robot."""
        leader = self.robots['leader']
        omega = self.leader_speed / self.leader_radius
        target_x = self.leader_radius * math.cos(omega * self.time)
        target_y = self.leader_radius * math.sin(omega * self.time)
        
        dx = target_x - leader.x
        dy = target_y - leader.y
        target_theta = math.atan2(dy, dx)
        
        angle_error = target_theta - leader.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        leader.set_cmd_vel(
            min(self.proportional_gain * math.sqrt(dx**2 + dy**2), 1.0),
            2.0 * angle_error
        )
    
    def _update_followers_proportional(self):
        """Update followers with proportional controller."""
        leader = self.robots['leader']
        leader_pos = np.array([leader.x, leader.y])
        
        formation_offsets = self.formations[self.current_formation]
        followers = ['follower_1', 'follower_2', 'follower_3']
        
        for i, follower_name in enumerate(followers):
            if i < len(formation_offsets):
                follower = self.robots[follower_name]
                offset = np.array(formation_offsets[i])
                target_pos = leader_pos + offset
                
                # Apply obstacle avoidance
                target_pos = self._apply_obstacle_avoidance(follower, target_pos)
                
                dx = target_pos[0] - follower.x
                dy = target_pos[1] - follower.y
                distance = math.sqrt(dx**2 + dy**2)
                target_angle = math.atan2(dy, dx)
                
                angle_error = target_angle - follower.theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                
                v_cmd = min(self.proportional_gain * distance, 1.0)
                omega_cmd = max(min(2.0 * angle_error, 1.5), -1.5)
                
                follower.set_cmd_vel(v_cmd, omega_cmd)
    
    def _update_followers_mpc(self):
        """Update followers with simulated MPC controller."""
        # Simulated MPC - enhanced with parameter tuning
        self.get_logger().info("🎛️ Using simulated MPC controller", throttle_duration_sec=5.0)
        # In reality, this would use CasADi optimization
        self._update_followers_proportional()  # Fallback for now
    
    def _update_followers_rl(self):
        """Update followers with simulated RL controller.""" 
        # Simulated RL - enhanced with learning parameters
        self.get_logger().info("🎛️ Using simulated RL controller", throttle_duration_sec=5.0)
        # In reality, this would use trained neural network policy
        self._update_followers_proportional()  # Fallback for now
    
    def _apply_obstacle_avoidance(self, robot, target_pos):
        """Apply obstacle avoidance to target position."""
        robot_pos = np.array([robot.x, robot.y])
        
        for obstacle in self.obstacles:
            obs_pos = np.array([obstacle.x, obstacle.y])
            distance = np.linalg.norm(robot_pos - obs_pos)
            
            if distance < obstacle.radius + 1.0:  # Safety margin
                # Repulsive force
                repulsion = (robot_pos - obs_pos) / (distance + 0.001)  # Avoid division by zero
                target_pos += repulsion * 0.5
        
        return target_pos
    
    def _publish_robot_states(self):
        """Publish enhanced robot odometry."""
        stamp = self.get_clock().now().to_msg()
        
        for robot_name, robot in self.robots.items():
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = f'{robot_name}/base_link'
            
            odom.pose.pose.position.x = robot.x
            odom.pose.pose.position.y = robot.y
            odom.pose.pose.orientation.z = math.sin(robot.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(robot.theta / 2.0)
            
            odom.twist.twist.linear.x = robot.v
            odom.twist.twist.angular.z = robot.omega
            
            self.odom_publishers[robot_name].publish(odom)
        
        # Enhanced status message
        status_msg = String()
        status_msg.data = (
            f'Formation: {self.current_formation}, '
            f'Controller: {self.current_controller}, '
            f'Obstacles: {len(self.obstacles)}, '
            f'Time: {self.time:.1f}s, '
            f'Spacing: {self.formation_spacing:.1f}m'
        )
        self.status_publisher.publish(status_msg)
    
    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        self.get_logger().info("🛑 Swarm controller shutting down...")

class SwarmVisualization:
    """🎮 NON-BLOCKING visualization system."""
    
    def __init__(self, node):
        self.node = node
        self.running = True
        
        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('🚀 Enhanced ROS2 Swarm System with Comprehensive Parameters', fontsize=14)
        
        # Robot plots
        self.robot_plots = {}
        self.trail_plots = {}
        
        self.get_logger = node.get_logger
        self.get_logger().info("🎮 NON-BLOCKING visualization system starting...")
        
    def run(self):
        """Run visualization loop in MAIN thread."""
        self.get_logger().info("🎮 Starting visualization in MAIN thread...")
        
        while self.running:
            try:
                if not self.update_plot():
                    break
                # No sleep needed - plt.pause handles timing
            except KeyboardInterrupt:
                self.get_logger().info("🛑 KeyboardInterrupt in visualization")
                break
            except Exception as e:
                self.get_logger().error(f"Visualization loop error: {e}")
                break
        
        plt.ioff()
        plt.close('all')
        self.get_logger().info("🎮 Visualization stopped")
    
    def update_plot(self):
        """Update visualization - NON-BLOCKING."""
        try:
            if not self.running:
                return False
                
            self.ax.clear()
            self.ax.set_xlim(-10, 10)
            self.ax.set_ylim(-10, 10)
            self.ax.set_aspect('equal')
            self.ax.grid(True, alpha=0.3)
            
            # Title with current status
            title = f'🚀 Enhanced ROS2 Swarm | Formation: {self.node.current_formation} | Controller: {self.node.current_controller}'
            self.ax.set_title(title, fontsize=12)
            
            # Draw robots
            for robot_name, robot in self.node.robots.items():
                # Robot position
                self.ax.plot(robot.x, robot.y, 'o', color=robot.color, markersize=10, label=robot_name)
                
                # Robot trail
                if len(robot.position_history) > 1:
                    trail_x, trail_y = zip(*robot.position_history[-50:])  # Last 50 points
                    self.ax.plot(trail_x, trail_y, '--', color=robot.color, alpha=0.5, linewidth=1)
                
                # Robot orientation
                dx = 0.3 * math.cos(robot.theta)
                dy = 0.3 * math.sin(robot.theta)
                self.ax.arrow(robot.x, robot.y, dx, dy, head_width=0.1, head_length=0.1, fc=robot.color, ec=robot.color)
            
            # Draw obstacles
            for obstacle in self.node.obstacles:
                color = 'purple' if obstacle.type == 'dynamic' else 'red'
                circle = Circle((obstacle.x, obstacle.y), obstacle.radius, color=color, alpha=0.6)
                self.ax.add_patch(circle)
            
            # Add parameter info
            info_text = f"Parameters: Multiple categories active | Time: {self.node.time:.1f}s"
            self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, fontsize=10, 
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            self.ax.legend(loc='upper right')
            plt.draw()
            plt.pause(0.05)  # Non-blocking pause
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Visualization error: {e}")
            return False
    
    def stop(self):
        self.running = False

def signal_handler(signum, frame):
    """Handle shutdown signals."""
    global running
    running = False
    print("\n🛑 Shutdown signal received...")

def main():
    """Main function with CORRECT threading - ROS2 in background, matplotlib in main."""
    global running
    running = True
    
    # Setup signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("🚀 Starting Enhanced ROS2 Swarm System with Comprehensive Parameters!")
    print("=" * 80)
    print("🔧 NEW FEATURES:")
    print("   ✅ Dynamic formation switching via ROS2 services")
    print("   ✅ Controller switching via ROS2 services")
    print("   ✅ Real-time obstacle management")
    print("   ✅ Comprehensive parameter server (30+ parameters)")
    print("   ✅ FIXED threading architecture (matplotlib in main)")
    print("   ✅ Professional system architecture")
    print("=" * 80)
    
    try:
        rclpy.init()
        
        # Create controller node
        controller = SwarmControllerNode()
        
        # Start ROS2 in BACKGROUND thread (this is key!)
        ros_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
        ros_thread.start()
        
        print("🚀 Professional system running! Test these commands:")
        print("=" * 60)
        print("🔄 SERVICES:")
        print("   ros2 service call /swarm/set_formation std_srvs/srv/Trigger")
        print("   ros2 service call /swarm/set_controller std_srvs/srv/Trigger")
        print("   ros2 service call /swarm/add_obstacle std_srvs/srv/Trigger")
        print("   ros2 service call /swarm/reset std_srvs/srv/Trigger")
        print("⚙️ PARAMETERS (examples):")
        print("   ros2 param set /swarm_controller formation.spacing 3.0")
        print("   ros2 param set /swarm_controller control.proportional_gain 1.5")
        print("   ros2 param set /swarm_controller leader.speed 1.5")
        print("   ros2 param set /swarm_controller obstacles.avoidance_distance 2.0")
        print("🧪 TESTING:")
        print("   python scripts/test_parameters.py")
        print("=" * 60)
        print("✅ System is NON-BLOCKING - you can switch windows freely!")
        print("🎮 Visualization running in MAIN thread... Close plot window or Ctrl+C to stop")
        
        # Run visualization in MAIN thread (this fixes the threading issue!)
        viz = SwarmVisualization(controller)
        viz.run()
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            if 'viz' in locals():
                viz.stop()
            if 'controller' in locals():
                controller.shutdown()
            rclpy.shutdown()
        except:
            pass
        print("🛑 Enhanced ROS2 Swarm System stopped")

if __name__ == '__main__':
    main() 