#!/usr/bin/env python3
"""
Unified ROS2 Swarm System Node

This node integrates all features from the Python demo:
- Formation Control: Triangle, Line, Circle, V-Shape
- Computer Vision: Leader detection and tracking
- Obstacle Avoidance: Static and dynamic obstacles
- Robot Collision Prevention: Prevents robots from passing through each other
- Multiple Controllers: Proportional and MPC
- Real-time Visualization: RViz markers and camera view

ROS2 Topics:
- /swarm/leader/pose - Leader position and orientation
- /swarm/followers/poses - Follower positions and orientations
- /swarm/formation_targets - Target positions for each follower
- /swarm/obstacles - Obstacle positions and properties
- /swarm/camera/image - Synthetic camera view
- /swarm/status - System status and parameters

Services:
- /swarm/set_formation - Change formation type
- /swarm/toggle_vision - Enable/disable vision system
- /swarm/toggle_obstacles - Enable/disable obstacle avoidance
- /swarm/add_obstacle - Add new obstacle
- /swarm/set_controller - Change controller type
- /swarm/toggle_vision_detection - Enable/disable vision detection
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, Pose
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from typing import List, Dict, Tuple, Optional, Any
import cv2
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Import our core logic
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../demos/python'))
from swarm_core import Robot, ProportionalController, get_formation_targets, VisionSystem, Obstacle, calculate_obstacle_avoidance, update_dynamic_obstacles, MultiRobotMPCController

# Import advanced controllers
from controllers import MPCController, EnhancedObstacleAvoidance, PerformanceMonitor

# Import enhanced vision system
from vision import EnhancedVisionSystemCore

# Import interactive control and data logging components
import json
import csv
import os
from datetime import datetime
from collections import deque
import threading
import queue
from dataclasses import dataclass
from enum import Enum

# Import performance plotter
from plotter import PerformancePlotter


class ControlMode(Enum):
    """System control modes"""
    NORMAL = 0
    EMERGENCY_STOP = 1
    SAFE_MODE = 2
    TEST_MODE = 3
    MANUAL_CONTROL = 4


class LogLevel(Enum):
    """Log levels for different types of data"""
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4


@dataclass
class SystemEvent:
    """System event structure"""
    timestamp: float
    event_type: str
    severity: LogLevel
    description: str
    robot_id: Optional[str] = None
    data: Dict[str, Any] = None


class UnifiedSwarmROS2Node(Node):
    """
    Unified ROS2 Swarm System Node
    Integrates all features from the Python demo into ROS2
    """
    
    def __init__(self):
        super().__init__('unified_swarm_system')
        
        # Declare parameters directly
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('leader_speed', 1.2)
        self.declare_parameter('leader_radius', 8.0)
        self.declare_parameter('num_followers', 3)
        
        # Controller parameters
        self.declare_parameter('kp_distance', 2.5)
        self.declare_parameter('kp_angle', 4.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        
        # Collision avoidance parameters
        self.declare_parameter('min_robot_distance', 1.2)
        self.declare_parameter('obstacle_detection_range', 3.0)
        self.declare_parameter('obstacle_safety_distance', 1.5)
        
        # Vision parameters
        self.declare_parameter('camera_resolution', [640, 480])
        self.declare_parameter('camera_fov', [-10.0, 10.0, -10.0, 10.0])
        
        # Interactive control parameters
        self.declare_parameter('interactive_control_enabled', True)
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('parameter_validation_enabled', True)
        
        # Data logging parameters
        self.declare_parameter('data_logging_enabled', True)
        self.declare_parameter('log_directory', 'logs')
        self.declare_parameter('max_log_size_mb', 100.0)
        self.declare_parameter('log_retention_days', 7)
        self.declare_parameter('metrics_buffer_size', 10000)
        
        # Initialize system state
        self.initialize_system()
        
        # Setup ROS2 publishers and subscribers
        self.setup_ros_interface()
        
        # Setup static transform broadcaster for map frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publish_static_transform()
        
        # Setup timers
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_system
        )
        
        self.publish_timer = self.create_timer(0.1, self.publish_data)
        
        self.performance_timer = self.create_timer(1.0, self.publish_performance_metrics)
        
        # Interactive control and data logging timers
        self.control_timer = self.create_timer(1.0, self.update_interactive_control)
        self.logging_timer = self.create_timer(2.0, self.update_data_logging)
        self.analysis_timer = self.create_timer(5.0, self.perform_analysis)
        self.cleanup_timer = self.create_timer(3600.0, self.cleanup_old_logs)  # Every hour
        
        # Start background threads for data processing
        self.start_background_threads()
        
        self.get_logger().info("🚀 Unified Swarm ROS2 System initialized!")
        self.get_logger().info("📡 Available services:")
        self.get_logger().info("   /swarm/set_formation")
        self.get_logger().info("   /swarm/toggle_vision")
        self.get_logger().info("   /swarm/toggle_obstacles")
        self.get_logger().info("   /swarm/add_obstacle")
        self.get_logger().info("   /swarm/set_controller")
        self.get_logger().info("   /swarm/toggle_vision_detection")
        self.get_logger().info("🎛️ Interactive Control Services:")
        self.get_logger().info("   /swarm/control/set_parameter")
        self.get_logger().info("   /swarm/control/emergency_stop")
        self.get_logger().info("   /swarm/control/reset_parameters")
        self.get_logger().info("   /swarm/control/tune_controller")
        self.get_logger().info("📊 Data Logging Services:")
        self.get_logger().info("   /swarm/logging/export_data")
        self.get_logger().info("   /swarm/logging/generate_report")
        self.get_logger().info("   /swarm/logging/clear_logs")
        self.get_logger().info(f"🎛️ Interactive control enabled: {self.interactive_control_enabled}")
        self.get_logger().info(f"📊 Data logging enabled: {self.data_logging_enabled}")
        self.get_logger().info(f"🚨 Emergency stop enabled: {self.emergency_stop_enabled}")
    
    def initialize_system(self):
        """Initialize the swarm system"""
        # Initialize vision system
        self.enhanced_vision = EnhancedVisionSystemCore()
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.dt = 1.0 / self.update_rate
        self.time = 0.0
        
        # Initialize robots with distinct colors
        self.leader = Robot(0, 0, 0, robot_id=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(-2, -1.5, 0, robot_id=1, color='blue', marker='o', name='Follower_1'),
            Robot(-2, 1.5, 0, robot_id=2, color='green', marker='s', name='Follower_2'),
            Robot(-3, 0, 0, robot_id=3, color='orange', marker='^', name='Follower_3')
        ]
        
        # System state
        self.current_formation = "triangle"
        self.vision_enabled = True  # Enable vision system by default
        self.obstacles_enabled = True  # Enable obstacle avoidance by default
        self.robot_collision_avoidance = True
        self.frame_count = 0  # For tracking vision metrics
        
        # Controllers
        kp_distance = self.get_parameter('kp_distance').value
        kp_angle = self.get_parameter('kp_angle').value
        self.prop_controller = ProportionalController(kp_distance=kp_distance, kp_angle=kp_angle)
        
        # Advanced controllers
        self.mpc_controller = MPCController(num_robots=3, prediction_horizon=10, control_horizon=3, dt=self.dt)
        self.enhanced_obstacle_avoidance = EnhancedObstacleAvoidance(detection_range=5.0, safety_distance=1.5)
        self.performance_monitor = PerformanceMonitor()
        
        # Controller options
        self.controllers = ["Proportional", "MPC"]
        self.current_controller_idx = 0
        
        # Vision system
        self.vision_system = VisionSystem()
        self.camera_image = None
        self.cv_bridge = CvBridge()
        
        # Enhanced vision system for real detection
        self.detected_leader_pos = None
        self.detected_robot_positions = {}
        self.vision_detection_enabled = True
        
        # Leader trajectory
        self.leader_speed = self.get_parameter('leader_speed').value
        self.leader_radius = self.get_parameter('leader_radius').value
        self.leader_center = np.array([0.0, 0.0])
        
        # Obstacles with diverse shapes, sizes, and colors
        self.static_obstacles = [
            Obstacle(x=3, y=2, radius=1.2, color='red', name='Static_1'),
            Obstacle(x=-6, y=-3, radius=0.8, color='blue', name='Static_2'),
            Obstacle(x=1, y=-4, radius=1.5, color='green', name='Static_3'),
            Obstacle(x=5, y=5, radius=0.6, color='orange', name='Static_4'),
            Obstacle(x=-8, y=6, radius=1.0, color='purple', name='Static_5'),
            Obstacle(x=0, y=8, radius=0.9, color='brown', name='Static_6'),
            Obstacle(x=-4, y=0, radius=1.3, color='darkred', name='Static_7'),
            Obstacle(x=8, y=-5, radius=0.7, color='darkblue', name='Static_8')
        ]
        self.dynamic_obstacles = [
            Obstacle(x=-2, y=5, radius=0.8, vx=0.6, vy=-0.3, color='purple', is_dynamic=True, name='Dynamic_1'),
            Obstacle(x=6, y=-2, radius=1.0, vx=-0.45, vy=0.9, color='orange', is_dynamic=True, name='Dynamic_2'),
            Obstacle(x=7, y=7, radius=0.9, vx=-0.5, vy=-0.4, color='green', is_dynamic=True, name='Dynamic_3'),
            Obstacle(x=-7, y=-7, radius=1.1, vx=0.4, vy=0.5, color='blue', is_dynamic=True, name='Dynamic_4')
        ]
        
        # Collision avoidance parameters
        self.min_robot_distance = self.get_parameter('min_robot_distance').value
        self.obstacle_detection_range = self.get_parameter('obstacle_detection_range').value
        self.obstacle_safety_distance = self.get_parameter('obstacle_safety_distance').value
        
        # Interactive control state
        self.interactive_control_enabled = self.get_parameter('interactive_control_enabled').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        self.parameter_validation_enabled = self.get_parameter('parameter_validation_enabled').value
        self.control_mode = ControlMode.NORMAL
        self.emergency_stop_active = False
        self.system_parameters = {}
        self.parameter_history = []
        self.control_commands = []
        self.system_status = {}
        self.parameter_changes_count = 0
        self.emergency_stops_count = 0
        self.last_save_time = time.time()
        
        # Data logging state
        self.data_logging_enabled = self.get_parameter('data_logging_enabled').value
        self.log_directory = self.get_parameter('log_directory').value
        self.max_log_size_mb = self.get_parameter('max_log_size_mb').value
        self.log_retention_days = self.get_parameter('log_retention_days').value
        self.metrics_buffer_size = self.get_parameter('metrics_buffer_size').value
        
        self.metrics_buffer = deque(maxlen=self.metrics_buffer_size)
        self.system_events = deque(maxlen=1000)
        self.performance_history = deque(maxlen=1000)
        self.formation_errors = deque(maxlen=1000)
        self.obstacle_distances = {}
        self.analysis_results = {}
        self.trend_data = {}
        
        # Threading for data processing
        self.data_queue = queue.Queue()
        self.lock = threading.Lock()
        
        # Performance plotter for generating charts and plots
        self.performance_plotter = PerformancePlotter(output_dir="performance_plots")
        self.plot_data_collection_enabled = True
        
        # Initialize interactive control parameters
        self.initialize_interactive_parameters()
        
        # Setup logging directory
        self.setup_logging_directory()
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers, subscribers, and services"""
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.leader_pose_pub = self.create_publisher(
            PoseStamped, '/swarm/leader/pose', qos_profile
        )
        
        self.followers_poses_pub = self.create_publisher(
            PoseStamped, '/swarm/followers/poses', qos_profile
        )
        
        self.formation_targets_pub = self.create_publisher(
            Point, '/swarm/formation_targets', qos_profile
        )
        
        self.obstacles_pub = self.create_publisher(
            MarkerArray, '/swarm/obstacles', qos_profile
        )
        
        self.camera_image_pub = self.create_publisher(
            Image, '/swarm/camera/image', qos_profile
        )
        
        self.status_pub = self.create_publisher(
            String, '/swarm/status', qos_profile
        )
        
        self.robot_markers_pub = self.create_publisher(
            MarkerArray, '/swarm/robot_markers', qos_profile
        )
        
        self.performance_metrics_pub = self.create_publisher(
            String, '/swarm/performance_metrics', qos_profile
        )
        
        # Services
        self.set_formation_srv = self.create_service(
            SetBool, '/swarm/set_formation', self.set_formation_callback
        )
        
        self.toggle_vision_srv = self.create_service(
            SetBool, '/swarm/toggle_vision', self.toggle_vision_callback
        )
        
        self.toggle_obstacles_srv = self.create_service(
            SetBool, '/swarm/toggle_obstacles', self.toggle_obstacles_callback
        )
        
        self.add_obstacle_srv = self.create_service(
            SetBool, '/swarm/add_obstacle', self.add_obstacle_callback
        )
        
        self.set_controller_srv = self.create_service(
            SetBool, '/swarm/set_controller', self.set_controller_callback
        )
        
        self.toggle_vision_detection_srv = self.create_service(
            SetBool, '/swarm/toggle_vision_detection', self.toggle_vision_detection_callback
        )
        
        # Interactive control services
        self.set_parameter_srv = self.create_service(
            SetBool, '/swarm/control/set_parameter', self.set_parameter_callback
        )
        
        self.emergency_stop_srv = self.create_service(
            SetBool, '/swarm/control/emergency_stop', self.emergency_stop_callback
        )
        
        self.reset_parameters_srv = self.create_service(
            SetBool, '/swarm/control/reset_parameters', self.reset_parameters_callback
        )
        
        self.tune_controller_srv = self.create_service(
            SetBool, '/swarm/control/tune_controller', self.tune_controller_callback
        )
        
        # Data logging services
        self.export_data_srv = self.create_service(
            SetBool, '/swarm/logging/export_data', self.export_data_callback
        )
        
        self.generate_report_srv = self.create_service(
            SetBool, '/swarm/logging/generate_report', self.generate_report_callback
        )
        
        self.clear_logs_srv = self.create_service(
            SetBool, '/swarm/logging/clear_logs', self.clear_logs_callback
        )
        
        # Interactive control publishers
        self.control_status_pub = self.create_publisher(
            String, '/swarm/control/status', qos_profile
        )
        
        self.parameter_values_pub = self.create_publisher(
            Float32MultiArray, '/swarm/control/parameters', qos_profile
        )
        
        self.emergency_status_pub = self.create_publisher(
            String, '/swarm/control/emergency', qos_profile
        )
        
        # Data logging publishers
        self.analysis_results_pub = self.create_publisher(
            String, '/swarm/analysis/results', qos_profile
        )
        
        self.alert_pub = self.create_publisher(
            String, '/swarm/alerts', qos_profile
        )
        
        # Enhanced vision system subscribers
        self.vision_detections_sub = self.create_subscription(
            Float32MultiArray, '/swarm/vision/detections', self.vision_detections_callback, qos_profile
        )
        
        self.vision_leader_pos_sub = self.create_subscription(
            Point, '/swarm/vision/leader_position', self.vision_leader_position_callback, qos_profile
        )
        
        self.vision_metrics_sub = self.create_subscription(
            Float32MultiArray, '/swarm/vision/metrics', self.vision_metrics_callback, qos_profile
        )
    
    def update_system(self):
        """Main system update loop"""
        self.update_leader()
        self.update_followers()
        
        # Update dynamic obstacles
        if self.obstacles_enabled:
            update_dynamic_obstacles(self.dynamic_obstacles, self.dt)
        
        # Collect data for plotting
        self.collect_plotting_data()
        
        self.publish_robot_transforms()
        self.time += self.dt
        self.frame_count += 1
    
    def update_leader(self):
        """Update leader position using velocity-based control toward a moving target, allowing obstacle avoidance to work."""
        # Use vision-based leader position if available, otherwise use synthetic
        if self.vision_detection_enabled and self.detected_leader_pos is not None:
            # Use detected leader position for formation control
            detected_x, detected_y = self.detected_leader_pos
            # Keep the synthetic motion for the actual leader, but use detected position for followers
            self.vision_leader_pos_for_formation = (detected_x, detected_y)
        else:
            # Fallback to synthetic leader position
            self.vision_leader_pos_for_formation = None
        
        # Compute the leader's moving target (e.g., a point on a circle)
        omega = self.leader_speed / self.leader_radius
        target_x = self.leader_center[0] + self.leader_radius * math.cos(omega * self.time)
        target_y = self.leader_center[1] + self.leader_radius * math.sin(omega * self.time)
        target_theta = omega * self.time + math.pi/2

        # Proportional control to move toward the target
        error_x = target_x - self.leader.x
        error_y = target_y - self.leader.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        kp_linear = 1.0
        kp_angular = 2.0
        v = kp_linear * distance_error
        desired_theta = math.atan2(error_y, error_x)
        angle_error = desired_theta - self.leader.theta
        # Normalize angle error
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        omega_cmd = kp_angular * angle_error

        # Limit velocities
        v = min(v, self.get_parameter('max_linear_vel').value)
        omega_cmd = max(-self.get_parameter('max_angular_vel').value, min(self.get_parameter('max_angular_vel').value, omega_cmd))

        # Apply obstacle avoidance (incremental, follower style)
        if self.obstacles_enabled:
            all_obstacles = self.static_obstacles + self.dynamic_obstacles
            avoidance_x, avoidance_y = calculate_obstacle_avoidance(self.leader, all_obstacles)
            self.leader.x += avoidance_x * self.dt * 0.5
            self.leader.y += avoidance_y * self.dt * 0.5

        # Apply robot-to-robot collision avoidance
        if self.robot_collision_avoidance:
            leader_avoidance_x, leader_avoidance_y = self.calculate_leader_avoidance()
            self.leader.x += leader_avoidance_x * self.dt * 0.3
            self.leader.y += leader_avoidance_y * self.dt * 0.3

        # Update leader state with velocity-based control
        self.leader.update(self.dt, v, omega_cmd)

        # Create camera image for vision system demonstration
        if self.vision_enabled:
            all_robots = [self.leader] + self.followers
            self.camera_image = self.vision_system.create_synthetic_image(all_robots)
            
            # Process camera image with enhanced vision system
            if self.camera_image is not None:
                detections = self.enhanced_vision.process_image(self.camera_image)
                
                # Update detected positions
                if detections:
                    for robot_id, detection in detections.items():
                        if robot_id == 0:  # Leader
                            self.detected_leader_pos = (detection.x, detection.y)
                        else:  # Followers
                            self.detected_robot_positions[robot_id] = (detection.x, detection.y)
                    
                    # Use vision-based leader position for formation if available
                    if 0 in detections:
                        self.vision_leader_pos_for_formation = (detections[0].x, detections[0].y)
                else:
                    # Fallback to synthetic positions if no detections
                    self.vision_leader_pos_for_formation = (self.leader.x, self.leader.y)
    
    def update_followers(self):
        """Update follower positions with advanced controllers and performance monitoring"""
        # Use vision-based leader position if available, otherwise use synthetic
        if self.vision_detection_enabled and self.vision_leader_pos_for_formation is not None:
            leader_pos = np.array(self.vision_leader_pos_for_formation)
            self.get_logger().debug(f"Using vision-based leader position: {leader_pos}")
        else:
            leader_pos = np.array([self.leader.x, self.leader.y])
            self.get_logger().debug(f"Using synthetic leader position: {leader_pos}")
        
        formation_targets = get_formation_targets(leader_pos, self.current_formation)
        
        # Collect robot states for advanced controllers
        robot_states = [follower.get_state() for follower in self.followers]
        
        # Initialize tracking variables
        total_formation_error = 0.0
        total_control_effort = 0.0
        collision_events = 0
        near_collision_events = 0
        
        if self.controllers[self.current_controller_idx] == "Proportional":
            for i, follower in enumerate(self.followers):
                target_pos = formation_targets[i]
                control = self.prop_controller.compute_control(follower.get_state(), target_pos)
                
                # Apply enhanced obstacle avoidance if enabled
                if self.obstacles_enabled:
                    all_obstacles = self.static_obstacles + self.dynamic_obstacles
                    avoidance_x, avoidance_y = self.enhanced_obstacle_avoidance.calculate_enhanced_avoidance(follower, all_obstacles)
                    follower.x += avoidance_x * self.dt * 0.5
                    follower.y += avoidance_y * self.dt * 0.5
                
                # Apply robot-to-robot collision avoidance
                if self.robot_collision_avoidance:
                    robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(follower, i)
                    follower.x += robot_avoidance_x * self.dt * 0.3
                    follower.y += robot_avoidance_y * self.dt * 0.3
                
                follower.update(self.dt, control[0], control[1])
                
                # Track performance metrics
                error = math.sqrt((follower.x - target_pos[0])**2 + (follower.y - target_pos[1])**2)
                total_formation_error += error
                total_control_effort += control[0]**2 + control[1]**2
                
                # Track formation error over time for this specific robot
                formation_error_data = {
                    'timestamp': time.time(),
                    'robot_id': f"follower_{i}",
                    'error': error,
                    'target_x': target_pos[0],
                    'target_y': target_pos[1],
                    'actual_x': follower.x,
                    'actual_y': follower.y
                }
                self.formation_errors.append(formation_error_data)
                
                # Check for collisions and near-collisions
                collision_detected, near_collision = self.check_collision_status(follower, i)
                if collision_detected:
                    collision_events += 1
                if near_collision:
                    near_collision_events += 1
        
        elif self.controllers[self.current_controller_idx] == "MPC":
            # Use advanced MPC controller
            start_time = time.time()
            controls = self.mpc_controller.compute_control(robot_states, formation_targets)
            solve_time = time.time() - start_time
            
            for i, (follower, control) in enumerate(zip(self.followers, controls)):
                # Apply enhanced obstacle avoidance if enabled
                if self.obstacles_enabled:
                    all_obstacles = self.static_obstacles + self.dynamic_obstacles
                    avoidance_x, avoidance_y = self.enhanced_obstacle_avoidance.calculate_enhanced_avoidance(follower, all_obstacles)
                    follower.x += avoidance_x * self.dt * 0.5
                    follower.y += avoidance_y * self.dt * 0.5
                
                # Apply robot-to-robot collision avoidance
                if self.robot_collision_avoidance:
                    robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(follower, i)
                    follower.x += robot_avoidance_x * self.dt * 0.3
                    follower.y += robot_avoidance_y * self.dt * 0.3
                
                follower.update(self.dt, control[0], control[1])
                
                # Track performance metrics
                target_pos = formation_targets[i]
                error = math.sqrt((follower.x - target_pos[0])**2 + (follower.y - target_pos[1])**2)
                total_formation_error += error
                total_control_effort += control[0]**2 + control[1]**2
                
                # Track formation error over time for this specific robot
                formation_error_data = {
                    'timestamp': time.time(),
                    'robot_id': f"follower_{i}",
                    'error': error,
                    'target_x': target_pos[0],
                    'target_y': target_pos[1],
                    'actual_x': follower.x,
                    'actual_y': follower.y
                }
                self.formation_errors.append(formation_error_data)
                
                # Check for collisions and near-collisions
                collision_detected, near_collision = self.check_collision_status(follower, i)
                if collision_detected:
                    collision_events += 1
                if near_collision:
                    near_collision_events += 1
            
            # Update performance monitor with MPC solve time
            avg_formation_error = total_formation_error / len(self.followers)
            avg_control_effort = total_control_effort / len(self.followers)
            self.performance_monitor.update_metrics(avg_formation_error, avg_control_effort, solve_time)
        else:
            # Update performance monitor for proportional controller
            avg_formation_error = total_formation_error / len(self.followers)
            avg_control_effort = total_control_effort / len(self.followers)
            self.performance_monitor.update_metrics(avg_formation_error, avg_control_effort, 0.0)
        
        # Track collision statistics
        collision_stats = {
            'timestamp': time.time(),
            'collision_events': collision_events,
            'near_collision_events': near_collision_events,
            'total_robots': len(self.followers) + 1  # Include leader
        }
        
        # Store collision statistics
        if not hasattr(self, 'collision_statistics'):
            self.collision_statistics = deque(maxlen=1000)
        self.collision_statistics.append(collision_stats)
        
        # Log collision events if they occur
        if collision_events > 0:
            collision_event = SystemEvent(
                timestamp=time.time(),
                event_type='collision_detected',
                severity=LogLevel.WARNING,
                description=f'Collision detected: {collision_events} events',
                data={'collision_events': collision_events, 'near_collision_events': near_collision_events}
            )
            self.log_event(collision_event)
    
    def check_collision_status(self, robot, robot_index):
        """Check if a robot is in collision or near-collision state"""
        collision_detected = False
        near_collision = False
        
        # Check collision with leader
        dx = robot.x - self.leader.x
        dy = robot.y - self.leader.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.5:  # Collision threshold
            collision_detected = True
        elif distance < self.min_robot_distance:  # Near collision threshold
            near_collision = True
        
        # Check collision with other followers
        for j, other_robot in enumerate(self.followers):
            if j != robot_index:
                dx = robot.x - other_robot.x
                dy = robot.y - other_robot.y
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < 0.5:  # Collision threshold
                    collision_detected = True
                elif distance < self.min_robot_distance:  # Near collision threshold
                    near_collision = True
        
        # Check collision with obstacles
        for obstacle in self.static_obstacles + self.dynamic_obstacles:
            dx = robot.x - obstacle.x
            dy = robot.y - obstacle.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < obstacle.radius:  # Collision with obstacle
                collision_detected = True
            elif distance < obstacle.radius + 0.5:  # Near collision with obstacle
                near_collision = True
        
        return collision_detected, near_collision
    
    def calculate_leader_avoidance(self):
        """Calculate avoidance forces to prevent leader from colliding with followers"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for follower in self.followers:
            dx = self.leader.x - follower.x
            dy = self.leader.y - follower.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.min_robot_distance and distance > 0.1:
                force_magnitude = 2.0 * (self.min_robot_distance - distance) / distance
                avoidance_x += force_magnitude * dx
                avoidance_y += force_magnitude * dy
        
        # Limit avoidance force
        max_avoidance = 1.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y
    
    def calculate_robot_avoidance(self, current_robot, robot_index):
        """Calculate avoidance forces to prevent robot-to-robot collisions"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        # Check collision with leader
        dx = current_robot.x - self.leader.x
        dy = current_robot.y - self.leader.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.min_robot_distance and distance > 0.1:
            force_magnitude = 2.0 * (self.min_robot_distance - distance) / distance
            avoidance_x += force_magnitude * dx
            avoidance_y += force_magnitude * dy
        
        # Check collision with other followers
        for j, other_robot in enumerate(self.followers):
            if j != robot_index:
                dx = current_robot.x - other_robot.x
                dy = current_robot.y - other_robot.y
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < self.min_robot_distance and distance > 0.1:
                    force_magnitude = 2.0 * (self.min_robot_distance - distance) / distance
                    avoidance_x += force_magnitude * dx
                    avoidance_y += force_magnitude * dy
        
        # Limit avoidance force
        max_avoidance = 1.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y
    
    def publish_data(self):
        """Publish all ROS2 data"""
        self.publish_robot_poses()
        self.publish_formation_targets()
        self.publish_obstacles()
        self.publish_camera_image()
        self.publish_robot_markers()
        self.publish_performance_metrics()
        self.publish_status()
    
    def publish_robot_poses(self):
        """Publish robot poses"""
        # Publish leader pose
        leader_pose = PoseStamped()
        leader_pose.header.stamp = self.get_clock().now().to_msg()
        leader_pose.header.frame_id = "map"
        leader_pose.pose.position.x = float(self.leader.x)
        leader_pose.pose.position.y = float(self.leader.y)
        leader_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = self.euler_to_quaternion(0, 0, self.leader.theta)
        leader_pose.pose.orientation.x = float(q[0])
        leader_pose.pose.orientation.y = float(q[1])
        leader_pose.pose.orientation.z = float(q[2])
        leader_pose.pose.orientation.w = float(q[3])
        
        self.leader_pose_pub.publish(leader_pose)
        
        # Publish follower poses
        for follower in self.followers:
            follower_pose = PoseStamped()
            follower_pose.header.stamp = self.get_clock().now().to_msg()
            follower_pose.header.frame_id = "map"
            follower_pose.pose.position.x = float(follower.x)
            follower_pose.pose.position.y = float(follower.y)
            follower_pose.pose.position.z = 0.0
            
            q = self.euler_to_quaternion(0, 0, follower.theta)
            follower_pose.pose.orientation.x = float(q[0])
            follower_pose.pose.orientation.y = float(q[1])
            follower_pose.pose.orientation.z = float(q[2])
            follower_pose.pose.orientation.w = float(q[3])
            
            self.followers_poses_pub.publish(follower_pose)
    
    def publish_formation_targets(self):
        """Publish formation target positions"""
        leader_pos = np.array([self.leader.x, self.leader.y])
        formation_targets = get_formation_targets(leader_pos, self.current_formation)
        
        for i, target in enumerate(formation_targets):
            target_point = Point()
            target_point.x = float(target[0])
            target_point.y = float(target[1])
            target_point.z = 0.0
            self.formation_targets_pub.publish(target_point)
    
    def publish_obstacles(self):
        """Publish obstacle markers with diverse shapes and colors"""
        marker_array = MarkerArray()
        
        # Color mapping
        color_map = {
            'red': (1.0, 0.0, 0.0),
            'blue': (0.0, 0.0, 1.0),
            'green': (0.0, 1.0, 0.0),
            'orange': (1.0, 0.5, 0.0),
            'purple': (0.5, 0.0, 0.5),
            'brown': (0.6, 0.3, 0.0),
            'darkred': (0.8, 0.0, 0.0),
            'darkblue': (0.0, 0.0, 0.8)
        }
        
        # Static obstacles with different shapes
        static_shapes = [Marker.CYLINDER, Marker.CUBE, Marker.SPHERE, Marker.CYLINDER, 
                        Marker.CUBE, Marker.SPHERE, Marker.CYLINDER, Marker.CUBE]
        
        for i, obstacle in enumerate(self.static_obstacles):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.ns = "obstacles"
            marker.id = i
            marker.type = static_shapes[i % len(static_shapes)]
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(obstacle.x)
            marker.pose.position.y = float(obstacle.y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale based on marker type
            if marker.type == Marker.CYLINDER:
                marker.scale.x = float(obstacle.radius * 2)
                marker.scale.y = float(obstacle.radius * 2)
                marker.scale.z = 0.3
            elif marker.type == Marker.CUBE:
                marker.scale.x = float(obstacle.radius * 1.5)
                marker.scale.y = float(obstacle.radius * 1.5)
                marker.scale.z = float(obstacle.radius * 1.5)
            else:  # SPHERE
                marker.scale.x = float(obstacle.radius * 2)
                marker.scale.y = float(obstacle.radius * 2)
                marker.scale.z = float(obstacle.radius * 2)
            
            # Set color
            color = color_map.get(obstacle.color, (0.8, 0.0, 0.0))
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        # Dynamic obstacles with different shapes
        dynamic_shapes = [Marker.SPHERE, Marker.CUBE, Marker.CYLINDER, Marker.SPHERE]
        
        for i, obstacle in enumerate(self.dynamic_obstacles):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.ns = "obstacles"
            marker.id = i + len(self.static_obstacles)
            marker.type = dynamic_shapes[i % len(dynamic_shapes)]
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(obstacle.x)
            marker.pose.position.y = float(obstacle.y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale based on marker type
            if marker.type == Marker.CYLINDER:
                marker.scale.x = float(obstacle.radius * 2)
                marker.scale.y = float(obstacle.radius * 2)
                marker.scale.z = 0.3
            elif marker.type == Marker.CUBE:
                marker.scale.x = float(obstacle.radius * 1.5)
                marker.scale.y = float(obstacle.radius * 1.5)
                marker.scale.z = float(obstacle.radius * 1.5)
            else:  # SPHERE
                marker.scale.x = float(obstacle.radius * 2)
                marker.scale.y = float(obstacle.radius * 2)
                marker.scale.z = float(obstacle.radius * 2)
            
            # Set color
            color = color_map.get(obstacle.color, (0.5, 0.0, 0.5))
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8  # Slightly more opaque for dynamic obstacles
            
            marker_array.markers.append(marker)
        
        self.obstacles_pub.publish(marker_array)
    
    def publish_camera_image(self):
        """Publish camera image"""
        if self.camera_image is not None and self.vision_enabled:
            try:
                ros_image = self.cv_bridge.cv2_to_imgmsg(self.camera_image, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_frame"
                self.camera_image_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish camera image: {e}")
    
    def publish_robot_markers(self):
        """Publish robot visualization markers"""
        marker_array = MarkerArray()
        
        # Leader marker
        leader_marker = Marker()
        leader_marker.header.stamp = self.get_clock().now().to_msg()
        leader_marker.header.frame_id = "map"
        leader_marker.ns = "robots"
        leader_marker.id = 0
        leader_marker.type = Marker.CYLINDER
        leader_marker.action = Marker.ADD
        
        leader_marker.pose.position.x = float(self.leader.x)
        leader_marker.pose.position.y = float(self.leader.y)
        leader_marker.pose.position.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.leader.theta)
        leader_marker.pose.orientation.x = float(q[0])
        leader_marker.pose.orientation.y = float(q[1])
        leader_marker.pose.orientation.z = float(q[2])
        leader_marker.pose.orientation.w = float(q[3])
        
        leader_marker.scale.x = 0.6
        leader_marker.scale.y = 0.6
        leader_marker.scale.z = 0.1
        
        leader_marker.color.r = 1.0
        leader_marker.color.g = 0.0
        leader_marker.color.b = 0.0
        leader_marker.color.a = 0.8
        
        marker_array.markers.append(leader_marker)
        
        # Follower markers
        colors = [(0, 0, 1), (0, 1, 0), (1, 0.5, 0)]  # Blue, Green, Orange
        for i, follower in enumerate(self.followers):
            follower_marker = Marker()
            follower_marker.header.stamp = self.get_clock().now().to_msg()
            follower_marker.header.frame_id = "map"
            follower_marker.ns = "robots"
            follower_marker.id = i + 1
            follower_marker.type = Marker.CYLINDER
            follower_marker.action = Marker.ADD
            
            follower_marker.pose.position.x = float(follower.x)
            follower_marker.pose.position.y = float(follower.y)
            follower_marker.pose.position.z = 0.0
            
            q = self.euler_to_quaternion(0, 0, follower.theta)
            follower_marker.pose.orientation.x = float(q[0])
            follower_marker.pose.orientation.y = float(q[1])
            follower_marker.pose.orientation.z = float(q[2])
            follower_marker.pose.orientation.w = float(q[3])
            
            follower_marker.scale.x = 0.5
            follower_marker.scale.y = 0.5
            follower_marker.scale.z = 0.1
            
            follower_marker.color.r = float(colors[i][0])
            follower_marker.color.g = float(colors[i][1])
            follower_marker.color.b = float(colors[i][2])
            follower_marker.color.a = 0.8
            
            marker_array.markers.append(follower_marker)
        
        self.robot_markers_pub.publish(marker_array)
    
    def publish_performance_metrics(self):
        """Publish performance metrics"""
        # Get recent performance data
        recent_performance = self.performance_monitor.get_recent_performance(window=10)
        
        if recent_performance:
            metrics_msg = String()
            metrics_msg.data = (
                f"Controller: {self.controllers[self.current_controller_idx]}, "
                f"Avg Formation Error: {recent_performance.get('avg_formation_error', 0.0):.3f}m, "
                f"Avg Control Effort: {recent_performance.get('avg_control_effort', 0.0):.3f}, "
                f"Avg Solve Time: {recent_performance.get('avg_solve_time', 0.0):.3f}s, "
                f"Runtime: {recent_performance.get('runtime', 0.0):.1f}s"
            )
            self.performance_metrics_pub.publish(metrics_msg)
    
    def publish_status(self):
        """Publish system status"""
        # Count detected robots
        detected_count = len(self.detected_robot_positions)
        vision_status = "DETECTED" if self.detected_leader_pos else "NO_DETECTION"
        
        status_msg = String()
        status_msg.data = (
            f"Formation: {self.current_formation}, "
            f"Controller: {self.controllers[self.current_controller_idx]}, "
            f"Vision: {'ON' if self.vision_enabled else 'OFF'}, "
            f"Vision Detection: {'ON' if self.vision_detection_enabled else 'OFF'} ({vision_status}), "
            f"Detected Robots: {detected_count}, "
            f"Obstacles: {'ON' if self.obstacles_enabled else 'OFF'}, "
            f"Robot Collision: {'ON' if self.robot_collision_avoidance else 'OFF'}, "
            f"Time: {self.time:.1f}s"
        )
        self.status_pub.publish(status_msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr
        
        return q
    
    def publish_robot_transforms(self):
        # Publish map -> leader/base_link
        self._publish_robot_tf(self.leader, 'leader')
        for i, follower in enumerate(self.followers, 1):
            self._publish_robot_tf(follower, f'follower_{i}')

    def _publish_robot_tf(self, robot, ns):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{ns}/base_link'
        t.transform.translation.x = robot.x
        t.transform.translation.y = robot.y
        t.transform.translation.z = 0.0
        # Convert theta to quaternion (yaw only, roll=pitch=0)
        yaw = robot.theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
    
    # Service callbacks
    def set_formation_callback(self, request, response):
        """Service callback to change formation"""
        formations = ["triangle", "line", "circle", "v_shape"]
        if request.data:
            # Cycle through formations
            current_idx = formations.index(self.current_formation)
            self.current_formation = formations[(current_idx + 1) % len(formations)]
            self.get_logger().info(f"🔄 Formation changed to: {self.current_formation}")
            response.success = True
            response.message = f"Formation set to {self.current_formation}"
        else:
            response.success = False
            response.message = f"Invalid request. Available formations: {formations}"
        return response
    
    def toggle_vision_callback(self, request, response):
        """Toggle synthetic vision system (camera image generation)"""
        self.vision_enabled = request.data
        status = "enabled" if self.vision_enabled else "disabled"
        self.get_logger().info(f"🎥 Synthetic vision system {status}")
        response.success = True
        response.message = f"Synthetic vision {status}"
        return response
    
    def toggle_obstacles_callback(self, request, response):
        """Service callback to toggle obstacles"""
        self.obstacles_enabled = request.data
        self.get_logger().info(f"🚧 Obstacles {'enabled' if self.obstacles_enabled else 'disabled'}")
        response.success = True
        response.message = f"Obstacles {'enabled' if self.obstacles_enabled else 'disabled'}"
        return response
    
    def add_obstacle_callback(self, request, response):
        """Service callback to add obstacle"""
        if request.data:
            # Add a random obstacle
            import random
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            radius = random.uniform(0.5, 1.5)
            if random.random() < 0.7:
                self.static_obstacles.append(Obstacle(x, y, radius, color='darkred'))
            else:
                vx = random.uniform(-0.8, 0.8)
                vy = random.uniform(-0.8, 0.8)
                self.dynamic_obstacles.append(Obstacle(x, y, radius, vx, vy, color='purple', is_dynamic=True))
            
            self.get_logger().info(f"🚧 Added obstacle at ({x:.1f}, {y:.1f})")
            response.success = True
            response.message = f"Added obstacle at ({x:.1f}, {y:.1f})"
        else:
            response.success = False
            response.message = "Invalid request"
        return response
    
    def set_controller_callback(self, request, response):
        """Service callback to change controller"""
        if request.data:
            # Cycle through controllers
            self.current_controller_idx = (self.current_controller_idx + 1) % len(self.controllers)
            controller_name = self.controllers[self.current_controller_idx]
            self.get_logger().info(f"🎛️ Controller switched to: {controller_name}")
            response.success = True
            response.message = f"Controller switched to {controller_name}"
        else:
            response.success = False
            response.message = "Invalid request"
        return response

    def publish_static_transform(self):
        """Publish static transform for map frame"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        
        # Identity transform (map and base_link are the same)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published static transform for map frame")

    def vision_detections_callback(self, msg: Float32MultiArray):
        """Callback for vision system detections"""
        if not self.vision_detection_enabled:
            return
        
        # Parse detection data: [robot_id, x, y, theta, confidence, area, ...]
        data = msg.data
        self.detected_robot_positions.clear()
        
        for i in range(0, len(data), 6):
            if i + 5 < len(data):
                robot_id = int(data[i])
                x = data[i + 1]
                y = data[i + 2]
                theta = data[i + 3]
                confidence = data[i + 4]
                area = data[i + 5]
                
                # Only use high confidence detections
                if confidence >= 2.0:  # MEDIUM or HIGH confidence
                    self.detected_robot_positions[robot_id] = (x, y, theta)
    
    def vision_leader_position_callback(self, msg: Point):
        """Callback for detected leader position"""
        if self.vision_detection_enabled:
            self.detected_leader_pos = (msg.x, msg.y)
    
    def vision_metrics_callback(self, msg: Float32MultiArray):
        """Callback for vision system metrics"""
        if len(msg.data) >= 6:
            detection_rate = msg.data[0]
            avg_confidence = msg.data[1]
            processing_time = msg.data[2]
            false_positives = msg.data[3]
            missed_detections = msg.data[4]
            total_detections = msg.data[5]
            
            # Log vision performance
            if self.frame_count % 100 == 0:  # Log every 100 frames
                self.get_logger().info(f"🎥 Vision Metrics: Rate={detection_rate:.2f}, "
                                     f"Confidence={avg_confidence:.2f}, "
                                     f"Detections={total_detections}")

    def toggle_vision_detection_callback(self, request, response):
        """Toggle real vision detection system"""
        self.vision_detection_enabled = request.data
        status = "enabled" if self.vision_detection_enabled else "disabled"
        self.get_logger().info(f"🔍 Real vision detection {status}")
        
        if not self.vision_detection_enabled:
            # Clear detected positions when disabling
            self.detected_leader_pos = None
            self.detected_robot_positions.clear()
            self.vision_leader_pos_for_formation = None
        
        response.success = True
        response.message = f"Vision detection {status}"
        return response

    # Interactive Control Methods
    def initialize_interactive_parameters(self):
        """Initialize interactive control parameters"""
        # Formation parameters
        self.add_parameter("formation_spacing", 2.0, "formation", 0.5, 5.0, 2.0, 0.1, "m")
        self.add_parameter("formation_tolerance", 0.3, "formation", 0.1, 1.0, 0.3, 0.05, "m")
        
        # Controller parameters
        self.add_parameter("kp_distance", 2.5, "controller", 0.1, 10.0, 2.5, 0.1, "")
        self.add_parameter("kp_angle", 4.0, "controller", 0.1, 10.0, 4.0, 0.1, "")
        self.add_parameter("max_linear_vel", 1.0, "controller", 0.1, 3.0, 1.0, 0.1, "m/s")
        self.add_parameter("max_angular_vel", 2.0, "controller", 0.1, 5.0, 2.0, 0.1, "rad/s")
        
        # Obstacle avoidance parameters
        self.add_parameter("obstacle_detection_range", 3.0, "obstacle_avoidance", 1.0, 10.0, 3.0, 0.2, "m")
        self.add_parameter("obstacle_safety_distance", 1.5, "obstacle_avoidance", 0.5, 3.0, 1.5, 0.1, "m")
        self.add_parameter("avoidance_strength", 2.0, "obstacle_avoidance", 0.5, 5.0, 2.0, 0.1, "")
        
        # Vision parameters
        self.add_parameter("vision_confidence_threshold", 0.6, "vision", 0.1, 1.0, 0.6, 0.05, "")
        self.add_parameter("vision_update_rate", 30.0, "vision", 5.0, 60.0, 30.0, 1.0, "Hz")
        
        # Safety parameters
        self.add_parameter("min_robot_distance", 1.2, "safety", 0.5, 3.0, 1.2, 0.1, "m")
        self.add_parameter("emergency_stop_distance", 0.5, "safety", 0.2, 1.0, 0.5, 0.05, "m")
        
        # Performance parameters
        self.add_parameter("system_update_rate", 20.0, "performance", 5.0, 50.0, 20.0, 1.0, "Hz")
        self.add_parameter("performance_monitoring_enabled", 1.0, "performance", 0.0, 1.0, 1.0, 1.0, "")
    
    def add_parameter(self, name: str, default_value: float, category: str, 
                     min_val: float, max_val: float, default_val: float, step: float, unit: str):
        """Add a new system parameter"""
        self.system_parameters[name] = {
            'value': default_value,
            'category': category,
            'range': {'min': min_val, 'max': max_val, 'default': default_val, 'step': step, 'unit': unit},
            'is_tunable': True,
            'last_modified': time.time()
        }
    
    def setup_logging_directory(self):
        """Setup logging directory structure"""
        try:
            os.makedirs(self.log_directory, exist_ok=True)
            os.makedirs(os.path.join(self.log_directory, 'metrics'), exist_ok=True)
            os.makedirs(os.path.join(self.log_directory, 'events'), exist_ok=True)
            os.makedirs(os.path.join(self.log_directory, 'exports'), exist_ok=True)
            os.makedirs(os.path.join(self.log_directory, 'visualizations'), exist_ok=True)
            
            self.get_logger().info(f"📁 Created logging directory structure in {self.log_directory}")
        except Exception as e:
            self.get_logger().error(f"Error creating logging directory: {e}")
    
    def start_background_threads(self):
        """Start background processing threads"""
        if self.data_logging_enabled:
            self.data_processing_thread = threading.Thread(target=self.data_processing_worker, daemon=True)
            self.data_processing_thread.start()
    
    def update_interactive_control(self):
        """Update interactive control system"""
        if not self.interactive_control_enabled:
            return
        
        try:
            # Process control commands
            self.process_control_commands()
            
            # Update system status
            self.update_system_status()
            
            # Check for emergency conditions
            self.check_emergency_conditions()
            
            # Apply parameter changes
            self.apply_parameter_changes()
            
            # Publish control data
            self.publish_control_data()
            
        except Exception as e:
            self.get_logger().error(f"Error in interactive control update: {e}")
    
    def update_data_logging(self):
        """Update data logging system"""
        if not self.data_logging_enabled:
            return
        
        try:
            # Log current system state
            self.log_system_metrics()
            
            # Process data queue
            self.process_data_queue()
            
        except Exception as e:
            self.get_logger().error(f"Error in data logging update: {e}")
    
    def process_control_commands(self):
        """Process pending control commands"""
        if not self.control_commands:
            return
        
        # Sort by priority (highest first)
        self.control_commands.sort(key=lambda cmd: cmd.get('priority', 0), reverse=True)
        
        # Process commands
        processed_commands = []
        for command in self.control_commands:
            if self.process_command(command):
                processed_commands.append(command)
        
        # Remove processed commands
        for cmd in processed_commands:
            self.control_commands.remove(cmd)
    
    def process_command(self, command: Dict[str, Any]) -> bool:
        """Process a single control command"""
        try:
            command_type = command.get('command_type')
            
            if command_type == "set_parameter":
                return self.execute_set_parameter(command.get('parameters', {}))
            elif command_type == "emergency_stop":
                return self.execute_emergency_stop(command.get('parameters', {}))
            elif command_type == "reset_parameters":
                return self.execute_reset_parameters(command.get('parameters', {}))
            elif command_type == "tune_controller":
                return self.execute_tune_controller(command.get('parameters', {}))
            else:
                self.get_logger().warn(f"Unknown command type: {command_type}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error processing command {command.get('command_type')}: {e}")
            return False
    
    def execute_set_parameter(self, params: Dict[str, Any]) -> bool:
        """Execute parameter setting command"""
        param_name = params.get("name")
        param_value = params.get("value")
        
        if param_name in self.system_parameters:
            old_value = self.system_parameters[param_name]['value']
            if self.set_parameter_value(param_name, param_value):
                self.get_logger().info(f"✅ Parameter '{param_name}' changed: {old_value} → {param_value}")
                return True
        
        return False
    
    def execute_emergency_stop(self, params: Dict[str, Any]) -> bool:
        """Execute emergency stop command"""
        stop_active = params.get("active", True)
        self.emergency_stop_active = stop_active
        
        if stop_active:
            self.control_mode = ControlMode.EMERGENCY_STOP
            self.emergency_stops_count += 1
            self.get_logger().warn("🚨 EMERGENCY STOP ACTIVATED!")
        else:
            self.control_mode = ControlMode.NORMAL
            self.get_logger().info("✅ Emergency stop deactivated")
        
        return True
    
    def execute_reset_parameters(self, params: Dict[str, Any]) -> bool:
        """Execute parameter reset command"""
        category_name = params.get("category", "all")
        
        if category_name == "all":
            # Reset all parameters to defaults
            for param_name, param in self.system_parameters.items():
                param['value'] = param['range']['default']
                param['last_modified'] = time.time()
            self.get_logger().info("🔄 All parameters reset to defaults")
        else:
            # Reset parameters in specific category
            reset_count = 0
            for param_name, param in self.system_parameters.items():
                if param['category'] == category_name:
                    param['value'] = param['range']['default']
                    param['last_modified'] = time.time()
                    reset_count += 1
            self.get_logger().info(f"🔄 Reset {reset_count} parameters in category '{category_name}'")
        
        return True
    
    def execute_tune_controller(self, params: Dict[str, Any]) -> bool:
        """Execute controller tuning command"""
        controller_type = params.get("controller", "proportional")
        tuning_factor = params.get("factor", 1.0)
        
        # Apply tuning to controller parameters
        if controller_type == "proportional":
            self.set_parameter_value("kp_distance", 
                                   self.system_parameters["kp_distance"]['value'] * tuning_factor)
            self.set_parameter_value("kp_angle", 
                                   self.system_parameters["kp_angle"]['value'] * tuning_factor)
            self.get_logger().info(f"🎛️ Tuned proportional controller by factor {tuning_factor}")
        elif controller_type == "obstacle_avoidance":
            self.set_parameter_value("avoidance_strength", 
                                   self.system_parameters["avoidance_strength"]['value'] * tuning_factor)
            self.get_logger().info(f"🎛️ Tuned obstacle avoidance by factor {tuning_factor}")
        
        return True
    
    def set_parameter_value(self, param_name: str, value: float) -> bool:
        """Set a parameter value with validation"""
        if param_name not in self.system_parameters:
            return False
        
        param = self.system_parameters[param_name]
        
        # Validate value
        if self.parameter_validation_enabled:
            if not (param['range']['min'] <= value <= param['range']['max']):
                self.get_logger().warn(f"Parameter '{param_name}' value {value} out of range "
                                     f"[{param['range']['min']}, {param['range']['max']}]")
                return False
        
        # Update parameter
        old_value = param['value']
        param['value'] = value
        param['last_modified'] = time.time()
        
        # Record change
        self.parameter_changes_count += 1
        self.parameter_history.append({
            'name': param_name,
            'old_value': old_value,
            'new_value': value,
            'timestamp': time.time(),
            'category': param['category']
        })
        
        # Limit history size
        if len(self.parameter_history) > 1000:
            self.parameter_history.pop(0)
        
        return True
    
    def check_emergency_conditions(self):
        """Check for emergency conditions"""
        if self.emergency_stop_active:
            # In emergency stop mode, continuously monitor
            pass
    
    def apply_parameter_changes(self):
        """Apply parameter changes to the system"""
        # Update controller parameters
        if "kp_distance" in self.system_parameters:
            self.prop_controller.kp_distance = self.system_parameters["kp_distance"]['value']
        if "kp_angle" in self.system_parameters:
            self.prop_controller.kp_angle = self.system_parameters["kp_angle"]['value']
        if "max_linear_vel" in self.system_parameters:
            self.max_linear_vel = self.system_parameters["max_linear_vel"]['value']
        if "max_angular_vel" in self.system_parameters:
            self.max_angular_vel = self.system_parameters["max_angular_vel"]['value']
        if "obstacle_detection_range" in self.system_parameters:
            self.obstacle_detection_range = self.system_parameters["obstacle_detection_range"]['value']
        if "obstacle_safety_distance" in self.system_parameters:
            self.obstacle_safety_distance = self.system_parameters["obstacle_safety_distance"]['value']
        if "min_robot_distance" in self.system_parameters:
            self.min_robot_distance = self.system_parameters["min_robot_distance"]['value']
    
    def update_system_status(self):
        """Update system status information"""
        self.system_status = {
            'control_mode': self.control_mode.value,
            'emergency_stop_active': self.emergency_stop_active,
            'parameter_changes_count': self.parameter_changes_count,
            'emergency_stops_count': self.emergency_stops_count,
            'active_parameters': len(self.system_parameters),
            'pending_commands': len(self.control_commands),
            'last_update': time.time()
        }
    
    def publish_control_data(self):
        """Publish control system data"""
        self.publish_control_status()
        self.publish_parameter_values()
        self.publish_emergency_status()
    
    def publish_control_status(self):
        """Publish control system status"""
        status_msg = String()
        status_data = {
            'mode': self.control_mode.value,
            'emergency_stop': self.emergency_stop_active,
            'parameter_changes': self.parameter_changes_count,
            'emergency_stops': self.emergency_stops_count,
            'timestamp': time.time()
        }
        status_msg.data = json.dumps(status_data)
        self.control_status_pub.publish(status_msg)
    
    def publish_parameter_values(self):
        """Publish current parameter values"""
        param_msg = Float32MultiArray()
        param_msg.data = []
        
        for param_name, param in self.system_parameters.items():
            param_msg.data.extend([
                float(hash(param_name) % 1000),  # Simple ID
                param['value'],
                float(hash(param['category']) % 100),  # Category ID
                param['last_modified']
            ])
        
        self.parameter_values_pub.publish(param_msg)
    
    def publish_emergency_status(self):
        """Publish emergency status"""
        emergency_msg = String()
        if self.emergency_stop_active:
            emergency_msg.data = "EMERGENCY_STOP_ACTIVE"
        else:
            emergency_msg.data = "NORMAL_OPERATION"
        self.emergency_status_pub.publish(emergency_msg)
    
    # Data Logging Methods
    def log_system_metrics(self):
        """Log current system metrics"""
        try:
            # Log robot positions
            for robot in [self.leader] + self.followers:
                metric = {
                    'timestamp': time.time(),
                    'robot_id': f"robot_{robot.robot_id}",
                    'metric_type': 'position',
                    'value': [robot.x, robot.y, 0.0],  # Use 0.0 for z
                    'metadata': {'orientation': robot.theta}
                }
                self.log_metric(metric)
            
            # Log formation errors with detailed tracking
            if hasattr(self, 'formation_errors') and self.formation_errors:
                # Log recent formation errors for each robot
                recent_errors = list(self.formation_errors)[-len(self.followers):]  # Last error for each robot
                for error_data in recent_errors:
                    metric = {
                        'timestamp': error_data['timestamp'],
                        'robot_id': error_data['robot_id'],
                        'metric_type': 'formation_error',
                        'value': error_data['error'],
                        'metadata': {
                            'target_x': error_data['target_x'],
                            'target_y': error_data['target_y'],
                            'actual_x': error_data['actual_x'],
                            'actual_y': error_data['actual_y']
                        }
                    }
                    self.log_metric(metric)
                
                # Log average formation error across all robots
                if len(recent_errors) > 0:
                    avg_error = sum(e['error'] for e in recent_errors) / len(recent_errors)
                    metric = {
                        'timestamp': time.time(),
                        'robot_id': 'system',
                        'metric_type': 'average_formation_error',
                        'value': avg_error,
                        'metadata': {'num_robots': len(recent_errors)}
                    }
                    self.log_metric(metric)
            
            # Log collision statistics
            if hasattr(self, 'collision_statistics') and self.collision_statistics:
                recent_collision_stats = list(self.collision_statistics)[-1]  # Most recent collision stats
                metric = {
                    'timestamp': recent_collision_stats['timestamp'],
                    'robot_id': 'system',
                    'metric_type': 'collision_statistics',
                    'value': {
                        'collision_events': recent_collision_stats['collision_events'],
                        'near_collision_events': recent_collision_stats['near_collision_events'],
                        'total_robots': recent_collision_stats['total_robots']
                    }
                }
                self.log_metric(metric)
                
                # Log collision rate over time
                if len(self.collision_statistics) > 1:
                    # Calculate collision rate over last 10 seconds
                    current_time = time.time()
                    recent_collisions = [s for s in self.collision_statistics if current_time - s['timestamp'] <= 10.0]
                    if recent_collisions:
                        total_collisions = sum(s['collision_events'] for s in recent_collisions)
                        collision_rate = total_collisions / 10.0  # Collisions per second
                        metric = {
                            'timestamp': current_time,
                            'robot_id': 'system',
                            'metric_type': 'collision_rate',
                            'value': collision_rate,
                            'metadata': {'time_window': 10.0, 'total_collisions': total_collisions}
                        }
                        self.log_metric(metric)
            
            # Log performance metrics
            if hasattr(self, 'performance_monitor'):
                perf_metrics = self.performance_monitor.get_recent_performance()
                metric = {
                    'timestamp': time.time(),
                    'robot_id': 'system',
                    'metric_type': 'performance',
                    'value': perf_metrics
                }
                self.log_metric(metric)
                
            # Log vision metrics
            if hasattr(self, 'enhanced_vision'):
                vision_metrics = self.enhanced_vision.get_performance_metrics()
                metric = {
                    'timestamp': time.time(),
                    'robot_id': 'vision_system',
                    'metric_type': 'vision_performance',
                    'value': vision_metrics
                }
                self.log_metric(metric)
                
            # Log controller-specific metrics
            current_controller = self.controllers[self.current_controller_idx]
            metric = {
                'timestamp': time.time(),
                'robot_id': 'system',
                'metric_type': 'controller_status',
                'value': {
                    'current_controller': current_controller,
                    'controller_index': self.current_controller_idx,
                    'total_controllers': len(self.controllers)
                }
            }
            self.log_metric(metric)
                
        except Exception as e:
            self.get_logger().error(f"Error logging system metrics: {e}")
    
    def log_metric(self, metric: Dict[str, Any]):
        """Log a metric data point"""
        try:
            # Add to buffer
            with self.lock:
                self.metrics_buffer.append(metric)
            
            # Add to processing queue
            self.data_queue.put(('metric', metric))
            
        except Exception as e:
            self.get_logger().error(f"Error logging metric: {e}")
    
    def log_event(self, event: SystemEvent):
        """Log a system event"""
        try:
            with self.lock:
                self.system_events.append(event)
            
            # Add to processing queue
            self.data_queue.put(('event', event))
            
        except Exception as e:
            self.get_logger().error(f"Error logging event: {e}")
    
    def data_processing_worker(self):
        """Background worker for data processing"""
        while rclpy.ok():
            try:
                # Get data from queue with timeout
                data_type, data = self.data_queue.get(timeout=1.0)
                
                if data_type == 'metric':
                    self.process_metric(data)
                elif data_type == 'event':
                    self.process_event(data)
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in data processing worker: {e}")
    
    def process_data_queue(self):
        """Process data from the queue"""
        try:
            while not self.data_queue.empty():
                data_type, data = self.data_queue.get_nowait()
                
                if data_type == 'metric':
                    self.process_metric(data)
                elif data_type == 'event':
                    self.process_event(data)
                    
        except Exception as e:
            self.get_logger().error(f"Error processing data queue: {e}")
    
    def process_metric(self, metric: Dict[str, Any]):
        """Process a metric data point"""
        try:
            # Check for alerts
            self.check_alert_conditions(metric)
            
            # Update trend analysis
            self.update_trend_analysis(metric)
            
            # Store to file if needed
            if len(self.metrics_buffer) % 100 == 0:  # Every 100 metrics
                self.save_metrics_to_file()
                
        except Exception as e:
            self.get_logger().error(f"Error processing metric: {e}")
    
    def process_event(self, event: SystemEvent):
        """Process a system event"""
        try:
            # Check for critical events
            if event.severity in [LogLevel.ERROR, LogLevel.CRITICAL]:
                self.handle_critical_event(event)
            
            # Store to file
            self.save_event_to_file(event)
            
        except Exception as e:
            self.get_logger().error(f"Error processing event: {e}")
    
    def check_alert_conditions(self, metric: Dict[str, Any]):
        """Check for alert conditions based on metrics"""
        try:
            metric_type = metric.get('metric_type')
            value = metric.get('value')
            
            if metric_type == 'formation_error':
                if value > 2.0:  # Formation error threshold
                    alert = {
                        'timestamp': metric['timestamp'],
                        'type': 'formation_error_high',
                        'severity': 'warning',
                        'robot_id': metric['robot_id'],
                        'value': value,
                        'threshold': 2.0
                    }
                    self.publish_alert(alert)
            
            elif metric_type == 'position':
                # Check for collision risk
                if isinstance(value, list) and len(value) >= 2:
                    x, y = value[0], value[1]
                    # Check distance to obstacles
                    for obstacle in self.static_obstacles + self.dynamic_obstacles:
                        distance = math.sqrt((x - obstacle.x)**2 + (y - obstacle.y)**2)
                        if distance < obstacle.radius + 0.5:  # Safety margin
                            alert = {
                                'timestamp': metric['timestamp'],
                                'type': 'obstacle_too_close',
                                'severity': 'critical',
                                'robot_id': metric['robot_id'],
                                'value': distance,
                                'threshold': obstacle.radius + 0.5
                            }
                            self.publish_alert(alert)
                            break
                        
        except Exception as e:
            self.get_logger().error(f"Error checking alert conditions: {e}")
    
    def publish_alert(self, alert: Dict[str, Any]):
        """Publish an alert"""
        try:
            alert_msg = String()
            alert_msg.data = json.dumps(alert)
            self.alert_pub.publish(alert_msg)
            
            self.get_logger().warn(f"🚨 Alert: {alert['type']} - Robot {alert['robot_id']}")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing alert: {e}")
    
    def update_trend_analysis(self, metric: Dict[str, Any]):
        """Update trend analysis for metrics"""
        try:
            metric_key = f"{metric['robot_id']}_{metric['metric_type']}"
            metric_value = metric['value']
            
            # Skip trend analysis for non-numeric values (like dictionaries)
            if isinstance(metric_value, dict):
                return  # Skip dictionary values for trend analysis
            
            if metric_key not in self.trend_data:
                self.trend_data[metric_key] = {
                    'values': deque(maxlen=100),
                    'timestamps': deque(maxlen=100),
                    'mean': 0.0,
                    'std': 0.0,
                    'trend': 'stable'
                }
            
            trend = self.trend_data[metric_key]
            
            # Handle different value types
            if isinstance(metric_value, list):
                # For position metrics, use the magnitude (distance from origin)
                if len(metric_value) >= 2:
                    value_for_analysis = math.sqrt(metric_value[0]**2 + metric_value[1]**2)
                else:
                    value_for_analysis = metric_value[0] if metric_value else 0.0
            elif isinstance(metric_value, (int, float)):
                value_for_analysis = float(metric_value)
            else:
                return  # Skip non-numeric values
            
            trend['values'].append(value_for_analysis)
            trend['timestamps'].append(metric['timestamp'])
            
            # Update statistics
            if len(trend['values']) > 10:
                values = list(trend['values'])
                trend['mean'] = np.mean(values)
                trend['std'] = np.std(values)
                
                # Simple trend detection
                if len(values) >= 20:
                    recent_mean = np.mean(values[-10:])
                    if recent_mean > trend['mean'] + trend['std']:
                        trend['trend'] = 'increasing'
                    elif recent_mean < trend['mean'] - trend['std']:
                        trend['trend'] = 'decreasing'
                    else:
                        trend['trend'] = 'stable'
                        
        except Exception as e:
            self.get_logger().error(f"Error updating trend analysis: {e}")
    
    def handle_critical_event(self, event: SystemEvent):
        """Handle critical system events"""
        try:
            self.get_logger().error(f"🚨 Critical event: {event.description}")
            
            # Take immediate action based on event type
            if "emergency_stop" in event.event_type.lower():
                # Log emergency stop event
                pass
            elif "collision" in event.event_type.lower():
                # Log collision event
                pass
            elif "communication_failure" in event.event_type.lower():
                # Log communication failure
                pass
                
        except Exception as e:
            self.get_logger().error(f"Error handling critical event: {e}")
    
    def perform_analysis(self):
        """Perform periodic analysis of collected data"""
        try:
            # Calculate performance statistics
            self.calculate_performance_statistics()
            
            # Generate analysis results
            self.generate_analysis_results()
            
            # Publish analysis results
            self.publish_analysis_results()
            
        except Exception as e:
            self.get_logger().error(f"Error performing analysis: {e}")
    
    def calculate_performance_statistics(self):
        """Calculate performance statistics from collected data"""
        try:
            with self.lock:
                if len(self.performance_history) > 0:
                    recent_metrics = list(self.performance_history)[-100:]  # Last 100 metrics
                    
                    stats = {
                        'avg_formation_error': np.mean([m.get('formation_error', 0) for m in recent_metrics]),
                        'avg_obstacle_avoidance': np.mean([m.get('obstacle_avoidance', 0) for m in recent_metrics]),
                        'avg_controller_response': np.mean([m.get('controller_response', 0) for m in recent_metrics]),
                        'avg_vision_accuracy': np.mean([m.get('vision_accuracy', 0) for m in recent_metrics]),
                        'avg_system_latency': np.mean([m.get('system_latency', 0) for m in recent_metrics])
                    }
                    
                    self.analysis_results['performance_stats'] = stats
                
                if len(self.formation_errors) > 0:
                    recent_errors = [e['error'] for e in list(self.formation_errors)[-50:]]
                    self.analysis_results['formation_error_stats'] = {
                        'mean': np.mean(recent_errors),
                        'std': np.std(recent_errors),
                        'max': np.max(recent_errors),
                        'min': np.min(recent_errors)
                    }
                    
        except Exception as e:
            self.get_logger().error(f"Error calculating performance statistics: {e}")
    
    def generate_analysis_results(self):
        """Generate comprehensive analysis results"""
        try:
            # Calculate collision statistics
            collision_stats = {}
            if hasattr(self, 'collision_statistics') and self.collision_statistics:
                recent_collisions = list(self.collision_statistics)[-100:]  # Last 100 collision records
                if recent_collisions:
                    total_collisions = sum(s['collision_events'] for s in recent_collisions)
                    total_near_collisions = sum(s['near_collision_events'] for s in recent_collisions)
                    collision_rate = total_collisions / len(recent_collisions) if recent_collisions else 0
                    near_collision_rate = total_near_collisions / len(recent_collisions) if recent_collisions else 0
                    
                    collision_stats = {
                        'total_collisions': total_collisions,
                        'total_near_collisions': total_near_collisions,
                        'collision_rate': collision_rate,
                        'near_collision_rate': near_collision_rate,
                        'safety_score': max(0, 100 - (total_collisions * 10 + total_near_collisions * 5))
                    }
            
            # Calculate formation error trends
            formation_trends = {}
            if hasattr(self, 'formation_errors') and self.formation_errors:
                recent_errors = list(self.formation_errors)[-50:]  # Last 50 formation errors
                if recent_errors:
                    # Group errors by robot
                    robot_errors = {}
                    for error in recent_errors:
                        robot_id = error['robot_id']
                        if robot_id not in robot_errors:
                            robot_errors[robot_id] = []
                        robot_errors[robot_id].append(error['error'])
                    
                    # Calculate trends for each robot
                    for robot_id, errors in robot_errors.items():
                        if len(errors) >= 10:
                            early_errors = errors[:len(errors)//2]
                            late_errors = errors[len(errors)//2:]
                            early_avg = np.mean(early_errors)
                            late_avg = np.mean(late_errors)
                            
                            if late_avg < early_avg * 0.9:
                                trend = 'improving'
                            elif late_avg > early_avg * 1.1:
                                trend = 'worsening'
                            else:
                                trend = 'stable'
                            
                            formation_trends[robot_id] = {
                                'current_avg': np.mean(errors),
                                'trend': trend,
                                'improvement': (early_avg - late_avg) / early_avg * 100 if early_avg > 0 else 0
                            }
            
            analysis = {
                'timestamp': time.time(),
                'performance_stats': self.analysis_results.get('performance_stats', {}),
                'formation_error_stats': self.analysis_results.get('formation_error_stats', {}),
                'collision_statistics': collision_stats,
                'formation_trends': formation_trends,
                'trend_analysis': self.trend_data,
                'system_health': self.assess_system_health(),
                'recommendations': self.generate_recommendations()
            }
            
            self.analysis_results['comprehensive'] = analysis
            
        except Exception as e:
            self.get_logger().error(f"Error generating analysis results: {e}")
    
    def assess_system_health(self) -> Dict[str, Any]:
        """Assess overall system health"""
        try:
            health_score = 100.0
            issues = []
            
            # Check formation errors
            if 'formation_error_stats' in self.analysis_results:
                mean_error = self.analysis_results['formation_error_stats']['mean']
                if mean_error > 2.0:
                    health_score -= 20
                    issues.append(f"High formation error: {mean_error:.2f}")
            
            # Check collision statistics
            if hasattr(self, 'collision_statistics') and self.collision_statistics:
                recent_collisions = list(self.collision_statistics)[-50:]  # Last 50 collision records
                if recent_collisions:
                    total_collisions = sum(s['collision_events'] for s in recent_collisions)
                    total_near_collisions = sum(s['near_collision_events'] for s in recent_collisions)
                    
                    if total_collisions > 5:
                        health_score -= 30
                        issues.append(f"High collision rate: {total_collisions} collisions")
                    elif total_collisions > 2:
                        health_score -= 15
                        issues.append(f"Moderate collision rate: {total_collisions} collisions")
                    
                    if total_near_collisions > 20:
                        health_score -= 10
                        issues.append(f"High near-collision rate: {total_near_collisions} events")
            
            # Check performance metrics
            if 'performance_stats' in self.analysis_results:
                stats = self.analysis_results['performance_stats']
                if stats.get('avg_obstacle_avoidance', 1.0) < 0.7:
                    health_score -= 15
                    issues.append("Poor obstacle avoidance performance")
                
                if stats.get('avg_vision_accuracy', 1.0) < 0.8:
                    health_score -= 10
                    issues.append("Low vision accuracy")
            
            # Check recent events
            with self.lock:
                recent_events = [e for e in list(self.system_events)[-10:] 
                               if e.severity in [LogLevel.ERROR, LogLevel.CRITICAL]]
                if recent_events:
                    health_score -= len(recent_events) * 5
                    issues.append(f"{len(recent_events)} recent critical events")
            
            # Check formation trends
            if 'formation_trends' in self.analysis_results.get('comprehensive', {}):
                formation_trends = self.analysis_results['comprehensive']['formation_trends']
                worsening_robots = [robot_id for robot_id, trend in formation_trends.items() 
                                  if trend['trend'] == 'worsening']
                if worsening_robots:
                    health_score -= len(worsening_robots) * 5
                    issues.append(f"Formation worsening for robots: {worsening_robots}")
            
            return {
                'score': max(0.0, health_score),
                'status': 'healthy' if health_score > 80 else 'warning' if health_score > 60 else 'critical',
                'issues': issues
            }
            
        except Exception as e:
            self.get_logger().error(f"Error assessing system health: {e}")
            return {'score': 0.0, 'status': 'unknown', 'issues': [str(e)]}
    
    def generate_recommendations(self) -> List[str]:
        """Generate recommendations based on analysis"""
        try:
            recommendations = []
            
            # Check formation performance
            if 'formation_error_stats' in self.analysis_results:
                mean_error = self.analysis_results['formation_error_stats']['mean']
                if mean_error > 2.0:
                    recommendations.append("Consider increasing formation control gains")
                    recommendations.append("Check for obstacles interfering with formation")
            
            # Check collision statistics
            if hasattr(self, 'collision_statistics') and self.collision_statistics:
                recent_collisions = list(self.collision_statistics)[-50:]
                if recent_collisions:
                    total_collisions = sum(s['collision_events'] for s in recent_collisions)
                    total_near_collisions = sum(s['near_collision_events'] for s in recent_collisions)
                    
                    if total_collisions > 5:
                        recommendations.append("🚨 CRITICAL: High collision rate detected - consider emergency stop")
                        recommendations.append("Increase collision avoidance strength")
                        recommendations.append("Reduce robot speeds to improve safety")
                    elif total_collisions > 2:
                        recommendations.append("⚠️ Moderate collision rate - increase safety margins")
                        recommendations.append("Review obstacle avoidance parameters")
                    
                    if total_near_collisions > 20:
                        recommendations.append("High near-collision rate - increase minimum robot distance")
                        recommendations.append("Consider reducing formation density")
            
            # Check formation trends
            if 'formation_trends' in self.analysis_results.get('comprehensive', {}):
                formation_trends = self.analysis_results['comprehensive']['formation_trends']
                worsening_robots = [robot_id for robot_id, trend in formation_trends.items() 
                                  if trend['trend'] == 'worsening']
                if worsening_robots:
                    recommendations.append(f"Formation performance worsening for robots: {worsening_robots}")
                    recommendations.append("Consider switching to MPC controller for better performance")
                    recommendations.append("Check for sensor issues on affected robots")
            
            # Check obstacle avoidance
            if 'performance_stats' in self.analysis_results:
                stats = self.analysis_results['performance_stats']
                if stats.get('avg_obstacle_avoidance', 1.0) < 0.7:
                    recommendations.append("Increase obstacle avoidance strength")
                    recommendations.append("Review obstacle detection parameters")
            
            # Check vision system
            if 'performance_stats' in self.analysis_results:
                stats = self.analysis_results['performance_stats']
                if stats.get('avg_vision_accuracy', 1.0) < 0.8:
                    recommendations.append("Improve lighting conditions for vision system")
                    recommendations.append("Consider recalibrating vision sensors")
            
            # General recommendations based on controller
            current_controller = self.controllers[self.current_controller_idx]
            if current_controller == "Proportional":
                recommendations.append("Consider switching to MPC controller for better performance")
            elif current_controller == "MPC":
                recommendations.append("MPC controller active - monitor computational load")
            
            return recommendations
            
        except Exception as e:
            self.get_logger().error(f"Error generating recommendations: {e}")
            return ["Error generating recommendations"]
    
    def publish_analysis_results(self):
        """Publish analysis results"""
        try:
            if 'comprehensive' in self.analysis_results:
                # Convert any deque to list for JSON serialization
                def convert_deques(obj):
                    if isinstance(obj, dict):
                        return {k: convert_deques(v) for k, v in obj.items()}
                    elif isinstance(obj, deque):
                        return list(obj)
                    elif isinstance(obj, list):
                        return [convert_deques(i) for i in obj]
                    else:
                        return obj
                results_data = convert_deques(self.analysis_results['comprehensive'])
                results_msg = String()
                results_msg.data = json.dumps(results_data)
                self.analysis_results_pub.publish(results_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing analysis results: {e}")
    
    def save_metrics_to_file(self):
        """Save metrics to file"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.log_directory, 'metrics', f'metrics_{timestamp}.json')
            
            with self.lock:
                metrics_data = []
                for metric in list(self.metrics_buffer)[-100:]:  # Last 100 metrics
                    metrics_data.append(metric)
            
            with open(filename, 'w') as f:
                json.dump(metrics_data, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Error saving metrics to file: {e}")
    
    def save_event_to_file(self, event: SystemEvent):
        """Save event to file"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d")
            filename = os.path.join(self.log_directory, 'events', f'events_{timestamp}.json')
            
            event_data = {
                'timestamp': event.timestamp,
                'event_type': event.event_type,
                'severity': event.severity.value,
                'description': event.description,
                'robot_id': event.robot_id,
                'data': event.data
            }
            
            # Append to file
            with open(filename, 'a') as f:
                f.write(json.dumps(event_data) + '\n')
                
        except Exception as e:
            self.get_logger().error(f"Error saving event to file: {e}")
    
    def cleanup_old_logs(self):
        """Clean up old log files"""
        try:
            cutoff_time = time.time() - (self.log_retention_days * 24 * 3600)
            
            for subdir in ['metrics', 'events', 'visualizations']:
                dir_path = os.path.join(self.log_directory, subdir)
                if os.path.exists(dir_path):
                    for filename in os.listdir(dir_path):
                        file_path = os.path.join(dir_path, filename)
                        if os.path.isfile(file_path):
                            if os.path.getmtime(file_path) < cutoff_time:
                                os.remove(file_path)
                                self.get_logger().info(f"🗑️ Cleaned up old log file: {filename}")
                                
        except Exception as e:
            self.get_logger().error(f"Error cleaning up old logs: {e}")
    
    # Interactive Control Service Callbacks
    def set_parameter_callback(self, request, response):
        """Service callback to set a parameter"""
        try:
            if request.data:
                # Add command to queue
                command = {
                    'command_type': "set_parameter",
                    'parameters': {"name": "kp_distance", "value": 3.0},  # Example
                    'timestamp': time.time(),
                    'priority': 1
                }
                self.control_commands.append(command)
                
                response.success = True
                response.message = "Parameter change command queued"
            else:
                response.success = False
                response.message = "Invalid request"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        try:
            command = {
                'command_type': "emergency_stop",
                'parameters': {"active": request.data},
                'timestamp': time.time(),
                'priority': 10  # High priority
            }
            self.control_commands.append(command)
            
            response.success = True
            response.message = f"Emergency stop {'activated' if request.data else 'deactivated'}"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def reset_parameters_callback(self, request, response):
        """Service callback to reset parameters"""
        try:
            command = {
                'command_type': "reset_parameters",
                'parameters': {"category": "all"},
                'timestamp': time.time(),
                'priority': 5
            }
            self.control_commands.append(command)
            
            response.success = True
            response.message = "Parameter reset command queued"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def tune_controller_callback(self, request, response):
        """Service callback to tune controller"""
        try:
            if request.data:
                command = {
                    'command_type': "tune_controller",
                    'parameters': {"controller": "proportional", "factor": 1.2},
                    'timestamp': time.time(),
                    'priority': 3
                }
                self.control_commands.append(command)
                
                response.success = True
                response.message = "Controller tuning command queued"
            else:
                response.success = False
                response.message = "Invalid request"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    # Data Logging Service Callbacks
    def export_data_callback(self, request, response):
        """Service callback to export data"""
        try:
            if request.data:
                export_path = self.export_data()
                response.success = True
                response.message = f"Data exported to {export_path}"
            else:
                response.success = False
                response.message = "Invalid request"
        except Exception as e:
            response.success = False
            response.message = f"Export failed: {str(e)}"
        
        return response
    
    def generate_report_callback(self, request, response):
        """Service callback to generate report"""
        try:
            if request.data:
                report_path = self.generate_report()
                response.success = True
                response.message = f"Report generated: {report_path}"
            else:
                response.success = False
                response.message = "Invalid request"
        except Exception as e:
            response.success = False
            response.message = f"Report generation failed: {str(e)}"
        
        return response
    
    def clear_logs_callback(self, request, response):
        """Service callback to clear logs"""
        try:
            if request.data:
                self.clear_logs()
                response.success = True
                response.message = "Logs cleared"
            else:
                response.success = False
                response.message = "Invalid request"
        except Exception as e:
            response.success = False
            response.message = f"Clear logs failed: {str(e)}"
        
        return response
    
    def export_data(self) -> str:
        """Export all collected data"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            export_dir = os.path.join(self.log_directory, 'exports', f'export_{timestamp}')
            os.makedirs(export_dir, exist_ok=True)
            
            with self.lock:
                # Export metrics
                metrics_file = os.path.join(export_dir, 'metrics.csv')
                with open(metrics_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'robot_id', 'metric_type', 'value', 'metadata'])
                    
                    for metric in self.metrics_buffer:
                        writer.writerow([
                            metric['timestamp'],
                            metric['robot_id'],
                            metric['metric_type'],
                            str(metric['value']),
                            json.dumps(metric.get('metadata', {}))
                        ])
                
                # Export system events
                events_file = os.path.join(export_dir, 'system_events.csv')
                with open(events_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'event_type', 'severity', 'description', 'robot_id', 'data'])
                    
                    for event in self.system_events:
                        writer.writerow([
                            event.timestamp,
                            event.event_type,
                            event.severity.value,
                            event.description,
                            event.robot_id or '',
                            json.dumps(event.data) if event.data else ''
                        ])
                
                # Export analysis results
                analysis_file = os.path.join(export_dir, 'analysis_results.json')
                with open(analysis_file, 'w') as f:
                    json.dump(self.analysis_results, f, indent=2)
            
            self.get_logger().info(f"📊 Data exported to {export_dir}")
            return export_dir
            
        except Exception as e:
            self.get_logger().error(f"Error exporting data: {e}")
            raise
    
    def generate_report(self) -> str:
        """Generate comprehensive report"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            report_file = os.path.join(self.log_directory, 'exports', f'report_{timestamp}.md')
            
            with open(report_file, 'w') as f:
                f.write("# Swarm System Performance Report\n\n")
                f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                # System health
                f.write("## System Health\n\n")
                health = self.analysis_results.get('comprehensive', {}).get('system_health', {})
                f.write(f"- **Health Score**: {health.get('score', 0):.1f}/100\n")
                f.write(f"- **Status**: {health.get('status', 'unknown')}\n")
                f.write(f"- **Issues**: {', '.join(health.get('issues', []))}\n\n")
                
                # Performance statistics
                f.write("## Performance Statistics\n\n")
                perf_stats = self.analysis_results.get('comprehensive', {}).get('performance_stats', {})
                for key, value in perf_stats.items():
                    f.write(f"- **{key.replace('_', ' ').title()}**: {value:.3f}\n")
                f.write("\n")
                
                # Formation error statistics
                f.write("## Formation Error Statistics\n\n")
                form_stats = self.analysis_results.get('comprehensive', {}).get('formation_error_stats', {})
                for key, value in form_stats.items():
                    f.write(f"- **{key.replace('_', ' ').title()}**: {value:.3f}\n")
                f.write("\n")
                
                # Recommendations
                f.write("## Recommendations\n\n")
                recommendations = self.analysis_results.get('comprehensive', {}).get('recommendations', [])
                for i, rec in enumerate(recommendations, 1):
                    f.write(f"{i}. {rec}\n")
                f.write("\n")
                
                # Data summary
                f.write("## Data Summary\n\n")
                with self.lock:
                    f.write(f"- **Total Metrics Logged**: {len(self.metrics_buffer)}\n")
                    f.write(f"- **Performance Records**: {len(self.performance_history)}\n")
                    f.write(f"- **System Events**: {len(self.system_events)}\n")
                    f.write(f"- **Formation Errors**: {len(self.formation_errors)}\n")
                
            self.get_logger().info(f"📋 Report generated: {report_file}")
            return report_file
            
        except Exception as e:
            self.get_logger().error(f"Error generating report: {e}")
            raise
    
    def clear_logs(self):
        """Clear all logged data"""
        try:
            with self.lock:
                self.metrics_buffer.clear()
                self.performance_history.clear()
                self.system_events.clear()
                self.formation_errors.clear()
            
            self.analysis_results.clear()
            self.trend_data.clear()
            
            self.get_logger().info("🗑️ All logged data cleared")
            
        except Exception as e:
            self.get_logger().error(f"Error clearing logs: {e}")

    def collect_plotting_data(self):
        """Collect data for performance plotting"""
        if not self.plot_data_collection_enabled:
            return
            
        try:
            current_time = time.time()
            
            # Collect robot positions
            for robot in [self.leader] + self.followers:
                self.performance_plotter.add_robot_position(
                    current_time, f"robot_{robot.robot_id}", 
                    robot.x, robot.y, robot.theta
                )
            
            # Collect formation errors
            if hasattr(self, 'formation_errors') and self.formation_errors:
                for error_data in list(self.formation_errors)[-5:]:  # Last 5 errors
                    self.performance_plotter.add_formation_error(
                        error_data['timestamp'],
                        error_data['robot_id'],
                        error_data['error']
                    )
            
            # Collect obstacle avoidance data
            for robot in [self.leader] + self.followers:
                min_distance = float('inf')
                for obstacle in self.static_obstacles + self.dynamic_obstacles:
                    distance = math.sqrt((robot.x - obstacle.x)**2 + (robot.y - obstacle.y)**2)
                    min_distance = min(min_distance, distance)
                
                if min_distance < float('inf'):
                    action = "avoid" if min_distance < self.obstacle_safety_distance else "normal"
                    self.performance_plotter.add_obstacle_avoidance(
                        current_time, f"robot_{robot.robot_id}", min_distance, action
                    )
            
            # Collect controller metrics
            if hasattr(self, 'performance_monitor'):
                perf_metrics = self.performance_monitor.get_recent_performance()
                if isinstance(perf_metrics, dict):
                    for metric_type, value in perf_metrics.items():
                        if isinstance(value, (int, float)):
                            self.performance_plotter.add_controller_metric(
                                current_time, "system", metric_type, float(value)
                            )
            
            # Collect vision metrics
            if hasattr(self, 'enhanced_vision'):
                vision_metrics = self.enhanced_vision.get_performance_metrics()
                if vision_metrics:
                    self.performance_plotter.add_vision_metric(
                        current_time,
                        vision_metrics.get('accuracy', 0.0),
                        vision_metrics.get('detection_count', 0),
                        vision_metrics.get('false_positives', 0)
                    )
            
            # Collect system performance (simulated)
            cpu_usage = 30.0 + 10.0 * np.sin(current_time * 0.1)  # Simulated CPU usage
            memory_usage = 50.0 + 5.0 * np.cos(current_time * 0.15)  # Simulated memory usage
            latency = 5.0 + 2.0 * np.random.random()  # Simulated latency
            self.performance_plotter.add_system_performance(
                current_time, cpu_usage, memory_usage, latency
            )
            
            # Collect energy metrics (simulated)
            for robot in [self.leader] + self.followers:
                energy_consumption = 0.1 + 0.05 * np.random.random()  # Simulated energy
                battery_level = 100.0 - (current_time * 0.01)  # Simulated battery drain
                self.performance_plotter.add_energy_metric(
                    current_time, f"robot_{robot.robot_id}", energy_consumption, battery_level
                )
                
        except Exception as e:
            self.get_logger().error(f"Error collecting plotting data: {e}")
    
    def generate_performance_plots(self):
        """Generate all performance plots and save to files"""
        try:
            self.get_logger().info("📊 Generating performance plots...")
            self.performance_plotter.generate_all_plots()
            self.get_logger().info("✅ Performance plots generated successfully!")
        except Exception as e:
            self.get_logger().error(f"Error generating performance plots: {e}")
    
    def shutdown_plotting(self):
        """Cleanup and generate final plots on shutdown"""
        try:
            self.get_logger().info("🔄 Generating final performance plots...")
            self.generate_performance_plots()
            self.get_logger().info("✅ Final plots generated successfully!")
        except Exception as e:
            self.get_logger().error(f"Error during plotting shutdown: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = UnifiedSwarmROS2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Generate final performance plots before shutdown
        node.shutdown_plotting()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 