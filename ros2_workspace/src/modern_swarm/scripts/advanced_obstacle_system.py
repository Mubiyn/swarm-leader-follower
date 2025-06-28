#!/usr/bin/env python3
"""
Advanced Obstacle System for ROS2 Swarm

This module provides advanced obstacle management capabilities including:
- Obstacle classification (static, dynamic, moving, dangerous)
- Trajectory prediction for dynamic obstacles
- Obstacle mapping and persistence
- Enhanced avoidance algorithms
- Performance metrics and analysis
"""

import numpy as np
import math
import time
from typing import List, Dict, Tuple, Optional, NamedTuple
from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool, String, Int32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from geometry_msgs.msg import TransformStamped


class ObstacleType(Enum):
    """Obstacle classification types"""
    STATIC = 0
    DYNAMIC = 1
    MOVING = 2
    DANGEROUS = 3
    UNKNOWN = 4


class ObstacleBehavior(Enum):
    """Obstacle behavior patterns"""
    STATIONARY = 0
    LINEAR = 1
    CIRCULAR = 2
    RANDOM = 3
    PURSUING = 4


@dataclass
class ObstaclePrediction:
    """Predicted obstacle trajectory"""
    obstacle_id: int
    predicted_positions: List[Tuple[float, float]]
    confidence: float
    prediction_time: float
    behavior_type: ObstacleBehavior


@dataclass
class ObstacleMetrics:
    """Obstacle interaction metrics"""
    obstacle_id: int
    interaction_count: int
    avoidance_success_rate: float
    average_avoidance_distance: float
    closest_approach: float
    total_interaction_time: float


class AdvancedObstacle:
    """Enhanced obstacle representation with classification and prediction"""
    
    def __init__(self, x: float, y: float, radius: float, obstacle_id: int, 
                 obstacle_type: ObstacleType = ObstacleType.UNKNOWN, 
                 vx: float = 0.0, vy: float = 0.0, 
                 color: str = 'red', name: str = 'obstacle'):
        self.x = x
        self.y = y
        self.radius = radius
        self.obstacle_id = obstacle_id
        self.obstacle_type = obstacle_type
        self.vx = vx
        self.vy = vy
        self.color = color
        self.name = name
        
        # Advanced properties
        self.behavior = ObstacleBehavior.STATIONARY
        self.danger_level = 1.0  # 0.0 (safe) to 1.0 (very dangerous)
        self.prediction_horizon = 10
        self.interaction_history = []
        self.avoidance_history = []
        self.last_update_time = time.time()
        
        # Classification confidence
        self.classification_confidence = 0.5
        self.behavior_confidence = 0.5
        
        # Performance metrics
        self.metrics = ObstacleMetrics(
            obstacle_id=obstacle_id,
            interaction_count=0,
            avoidance_success_rate=1.0,
            average_avoidance_distance=float('inf'),
            closest_approach=float('inf'),
            total_interaction_time=0.0
        )
    
    def update_position(self, dt: float):
        """Update obstacle position based on velocity and behavior"""
        old_x, old_y = self.x, self.y
        
        # Update based on behavior
        if self.behavior == ObstacleBehavior.STATIONARY:
            pass  # No movement
        elif self.behavior == ObstacleBehavior.LINEAR:
            self.x += self.vx * dt
            self.y += self.vy * dt
        elif self.behavior == ObstacleBehavior.CIRCULAR:
            # Circular motion around a center
            center_x, center_y = 0.0, 0.0
            radius = 5.0
            angular_vel = 0.5
            angle = angular_vel * time.time()
            self.x = center_x + radius * math.cos(angle)
            self.y = center_y + radius * math.sin(angle)
        elif self.behavior == ObstacleBehavior.RANDOM:
            # Random walk
            self.vx += np.random.normal(0, 0.1)
            self.vy += np.random.normal(0, 0.1)
            self.x += self.vx * dt
            self.y += self.vy * dt
        elif self.behavior == ObstacleBehavior.PURSUING:
            # Simple pursuing behavior (would need target robot)
            pass
        
        # Boundary checking
        if abs(self.x) > 15 or abs(self.y) > 15:
            self.vx *= -1
            self.y *= -1
        
        # Update interaction history
        self.interaction_history.append((self.x, self.y, time.time()))
        if len(self.interaction_history) > 100:
            self.interaction_history.pop(0)
        
        self.last_update_time = time.time()
    
    def classify_behavior(self):
        """Classify obstacle behavior based on movement history"""
        if len(self.interaction_history) < 10:
            return
        
        # Extract recent positions
        recent_positions = self.interaction_history[-10:]
        positions = np.array([(x, y) for x, y, _ in recent_positions])
        
        # Calculate movement statistics
        if len(positions) >= 2:
            movements = np.diff(positions, axis=0)
            movement_magnitudes = np.linalg.norm(movements, axis=1)
            avg_movement = np.mean(movement_magnitudes)
            
            # Classify based on movement patterns
            if avg_movement < 0.1:
                self.behavior = ObstacleBehavior.STATIONARY
                self.behavior_confidence = 0.9
            elif avg_movement > 0.5:
                # Check if movement is consistent (linear) or random
                movement_angles = np.arctan2(movements[:, 1], movements[:, 0])
                angle_variance = np.var(movement_angles)
                
                if angle_variance < 0.5:
                    self.behavior = ObstacleBehavior.LINEAR
                    self.behavior_confidence = 0.8
                else:
                    self.behavior = ObstacleBehavior.RANDOM
                    self.behavior_confidence = 0.7
    
    def predict_trajectory(self, prediction_steps: int = 10) -> ObstaclePrediction:
        """Predict future trajectory based on current behavior"""
        predicted_positions = []
        current_x, current_y = self.x, self.y
        current_vx, current_vy = self.vx, self.vy
        dt = 0.1
        
        for step in range(prediction_steps):
            if self.behavior == ObstacleBehavior.STATIONARY:
                predicted_positions.append((current_x, current_y))
            elif self.behavior == ObstacleBehavior.LINEAR:
                current_x += current_vx * dt
                current_y += current_vy * dt
                predicted_positions.append((current_x, current_y))
            elif self.behavior == ObstacleBehavior.CIRCULAR:
                center_x, center_y = 0.0, 0.0
                radius = 5.0
                angular_vel = 0.5
                angle = angular_vel * (time.time() + step * dt)
                pred_x = center_x + radius * math.cos(angle)
                pred_y = center_y + radius * math.sin(angle)
                predicted_positions.append((pred_x, pred_y))
            elif self.behavior == ObstacleBehavior.RANDOM:
                # Simple random walk prediction
                current_x += current_vx * dt + np.random.normal(0, 0.1)
                current_y += current_vy * dt + np.random.normal(0, 0.1)
                predicted_positions.append((current_x, current_y))
        
        return ObstaclePrediction(
            obstacle_id=self.obstacle_id,
            predicted_positions=predicted_positions,
            confidence=self.behavior_confidence,
            prediction_time=time.time(),
            behavior_type=self.behavior
        )
    
    def update_metrics(self, robot_distance: float, avoidance_success: bool):
        """Update obstacle interaction metrics"""
        self.metrics.interaction_count += 1
        self.metrics.closest_approach = min(self.metrics.closest_approach, robot_distance)
        
        # Update avoidance success rate
        if self.metrics.interaction_count > 0:
            if avoidance_success:
                self.metrics.avoidance_success_rate = (
                    (self.metrics.avoidance_success_rate * (self.metrics.interaction_count - 1) + 1.0) 
                    / self.metrics.interaction_count
                )
            else:
                self.metrics.avoidance_success_rate = (
                    (self.metrics.avoidance_success_rate * (self.metrics.interaction_count - 1)) 
                    / self.metrics.interaction_count
                )
        
        # Update average avoidance distance
        if self.metrics.average_avoidance_distance == float('inf'):
            self.metrics.average_avoidance_distance = robot_distance
        else:
            self.metrics.average_avoidance_distance = (
                (self.metrics.average_avoidance_distance * (self.metrics.interaction_count - 1) + robot_distance)
                / self.metrics.interaction_count
            )


class AdvancedObstacleSystem(Node):
    """
    Advanced obstacle management system with classification, prediction, and mapping
    """
    
    def __init__(self):
        super().__init__('advanced_obstacle_system')
        
        # Declare parameters
        self.declare_parameter('prediction_horizon', 10)
        self.declare_parameter('classification_confidence_threshold', 0.7)
        self.declare_parameter('obstacle_mapping_enabled', True)
        self.declare_parameter('prediction_enabled', True)
        self.declare_parameter('update_rate', 20.0)
        
        # Initialize parameters
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.classification_threshold = self.get_parameter('classification_confidence_threshold').value
        self.mapping_enabled = self.get_parameter('obstacle_mapping_enabled').value
        self.prediction_enabled = self.get_parameter('prediction_enabled').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Obstacle storage
        self.obstacles = {}  # obstacle_id -> AdvancedObstacle
        self.obstacle_predictions = {}
        self.obstacle_metrics = {}
        self.next_obstacle_id = 0
        
        # Mapping
        self.occupancy_grid = None
        self.grid_resolution = 0.5
        self.grid_size = 40  # 20m x 20m grid
        
        # Performance tracking
        self.total_obstacles_created = 0
        self.total_interactions = 0
        self.avoidance_success_rate = 1.0
        self.average_prediction_accuracy = 0.0
        
        # Setup ROS2 interface
        self.setup_ros_interface()
        
        # Setup timers
        self.setup_timers()
        
        # Initialize with some example obstacles
        self.initialize_example_obstacles()
        
        self.get_logger().info("🚧 Advanced Obstacle System initialized!")
        self.get_logger().info(f"📊 Prediction horizon: {self.prediction_horizon}")
        self.get_logger().info(f"🗺️ Mapping enabled: {self.mapping_enabled}")
        self.get_logger().info(f"🔮 Prediction enabled: {self.prediction_enabled}")
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers, subscribers, and services"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.obstacles_pub = self.create_publisher(
            MarkerArray, '/swarm/obstacles/advanced', qos_profile
        )
        
        self.predictions_pub = self.create_publisher(
            MarkerArray, '/swarm/obstacles/predictions', qos_profile
        )
        
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid, '/swarm/obstacles/occupancy_grid', qos_profile
        )
        
        self.obstacle_metrics_pub = self.create_publisher(
            Float32MultiArray, '/swarm/obstacles/metrics', qos_profile
        )
        
        self.obstacle_status_pub = self.create_publisher(
            String, '/swarm/obstacles/status', qos_profile
        )
        
        # Subscribers
        self.robot_poses_sub = self.create_subscription(
            PoseStamped, '/swarm/leader/pose', self.robot_pose_callback, qos_profile
        )
        
        # Services
        self.add_obstacle_srv = self.create_service(
            Bool, '/swarm/obstacles/add_advanced', self.add_obstacle_callback
        )
        
        self.classify_obstacles_srv = self.create_service(
            Bool, '/swarm/obstacles/classify', self.classify_obstacles_callback
        )
        
        self.predict_trajectories_srv = self.create_service(
            Bool, '/swarm/obstacles/predict', self.predict_trajectories_callback
        )
    
    def setup_timers(self):
        """Setup ROS2 timers"""
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_obstacles
        )
        
        self.publish_timer = self.create_timer(0.1, self.publish_data)
        
        self.metrics_timer = self.create_timer(1.0, self.publish_metrics)
    
    def initialize_example_obstacles(self):
        """Initialize with example obstacles for demonstration"""
        # Static obstacles
        self.add_obstacle(3, 2, 1.2, ObstacleType.STATIC, color='red', name='Static_1')
        self.add_obstacle(-6, -3, 0.8, ObstacleType.STATIC, color='blue', name='Static_2')
        self.add_obstacle(1, -4, 1.5, ObstacleType.STATIC, color='green', name='Static_3')
        
        # Dynamic obstacles
        self.add_obstacle(-2, 5, 0.8, ObstacleType.DYNAMIC, vx=0.6, vy=-0.3, 
                         color='purple', name='Dynamic_1')
        self.add_obstacle(6, -2, 1.0, ObstacleType.DYNAMIC, vx=-0.45, vy=0.9, 
                         color='orange', name='Dynamic_2')
        
        # Moving obstacles with different behaviors
        self.add_obstacle(7, 7, 0.9, ObstacleType.MOVING, vx=-0.5, vy=-0.4, 
                         color='green', name='Moving_1')
        self.add_obstacle(-7, -7, 1.1, ObstacleType.MOVING, vx=0.4, vy=0.5, 
                         color='blue', name='Moving_2')
        
        # Dangerous obstacles
        self.add_obstacle(0, 8, 0.9, ObstacleType.DANGEROUS, vx=0.8, vy=-0.6, 
                         color='darkred', name='Dangerous_1')
    
    def add_obstacle(self, x: float, y: float, radius: float, 
                    obstacle_type: ObstacleType, vx: float = 0.0, vy: float = 0.0,
                    color: str = 'red', name: str = 'obstacle') -> int:
        """Add a new obstacle to the system"""
        obstacle_id = self.next_obstacle_id
        self.next_obstacle_id += 1
        
        obstacle = AdvancedObstacle(
            x=x, y=y, radius=radius, obstacle_id=obstacle_id,
            obstacle_type=obstacle_type, vx=vx, vy=vy,
            color=color, name=name
        )
        
        self.obstacles[obstacle_id] = obstacle
        self.total_obstacles_created += 1
        
        self.get_logger().info(f"🚧 Added {obstacle_type.name} obstacle '{name}' at ({x:.1f}, {y:.1f})")
        return obstacle_id
    
    def update_obstacles(self):
        """Update all obstacles"""
        dt = 1.0 / self.update_rate
        
        for obstacle in self.obstacles.values():
            # Update position
            obstacle.update_position(dt)
            
            # Classify behavior
            obstacle.classify_behavior()
            
            # Generate predictions if enabled
            if self.prediction_enabled:
                prediction = obstacle.predict_trajectory(self.prediction_horizon)
                self.obstacle_predictions[obstacle.obstacle_id] = prediction
    
    def robot_pose_callback(self, msg: PoseStamped):
        """Callback for robot poses to track interactions"""
        robot_x = msg.pose.position.x
        robot_y = msg.pose.position.y
        
        # Check interactions with all obstacles
        for obstacle in self.obstacles.values():
            distance = math.sqrt((robot_x - obstacle.x)**2 + (robot_y - obstacle.y)**2)
            safe_distance = obstacle.radius + 1.5  # Robot safety margin
            
            if distance < safe_distance:
                # Record interaction
                avoidance_success = distance > obstacle.radius + 0.5
                obstacle.update_metrics(distance, avoidance_success)
                self.total_interactions += 1
    
    def add_obstacle_callback(self, request, response):
        """Service callback to add a new obstacle"""
        if request.data:
            # Add a random advanced obstacle
            import random
            
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            radius = random.uniform(0.5, 1.5)
            
            # Randomly choose obstacle type
            obstacle_types = list(ObstacleType)
            obstacle_type = random.choice(obstacle_types)
            
            # Set velocity based on type
            if obstacle_type == ObstacleType.STATIC:
                vx = vy = 0.0
            else:
                vx = random.uniform(-0.8, 0.8)
                vy = random.uniform(-0.8, 0.8)
            
            # Choose color based on type
            color_map = {
                ObstacleType.STATIC: 'gray',
                ObstacleType.DYNAMIC: 'blue',
                ObstacleType.MOVING: 'green',
                ObstacleType.DANGEROUS: 'red',
                ObstacleType.UNKNOWN: 'yellow'
            }
            color = color_map.get(obstacle_type, 'white')
            
            obstacle_id = self.add_obstacle(x, y, radius, obstacle_type, vx, vy, color)
            
            response.success = True
            response.message = f"Added {obstacle_type.name} obstacle (ID: {obstacle_id}) at ({x:.1f}, {y:.1f})"
        else:
            response.success = False
            response.message = "Invalid request"
        
        return response
    
    def classify_obstacles_callback(self, request, response):
        """Service callback to classify all obstacles"""
        if request.data:
            classified_count = 0
            for obstacle in self.obstacles.values():
                obstacle.classify_behavior()
                if obstacle.behavior_confidence > self.classification_threshold:
                    classified_count += 1
            
            response.success = True
            response.message = f"Classified {classified_count}/{len(self.obstacles)} obstacles"
        else:
            response.success = False
            response.message = "Invalid request"
        
        return response
    
    def predict_trajectories_callback(self, request, response):
        """Service callback to predict obstacle trajectories"""
        if request.data and self.prediction_enabled:
            prediction_count = 0
            for obstacle in self.obstacles.values():
                prediction = obstacle.predict_trajectory(self.prediction_horizon)
                self.obstacle_predictions[obstacle.obstacle_id] = prediction
                prediction_count += 1
            
            response.success = True
            response.message = f"Generated {prediction_count} trajectory predictions"
        else:
            response.success = False
            response.message = "Prediction disabled or invalid request"
        
        return response
    
    def publish_data(self):
        """Publish obstacle data"""
        self.publish_obstacles()
        self.publish_predictions()
        if self.mapping_enabled:
            self.publish_occupancy_grid()
    
    def publish_obstacles(self):
        """Publish obstacle markers"""
        marker_array = MarkerArray()
        marker_id = 0
        
        for obstacle in self.obstacles.values():
            # Create obstacle marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "advanced_obstacles"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.position.z = 0.1
            
            # Scale based on radius
            marker.scale.x = obstacle.radius * 2
            marker.scale.y = obstacle.radius * 2
            marker.scale.z = 0.2
            
            # Color based on type and danger level
            if obstacle.obstacle_type == ObstacleType.DANGEROUS:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif obstacle.obstacle_type == ObstacleType.MOVING:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obstacle.obstacle_type == ObstacleType.DYNAMIC:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        if marker_array.markers:
            self.obstacles_pub.publish(marker_array)
    
    def publish_predictions(self):
        """Publish obstacle trajectory predictions"""
        if not self.prediction_enabled:
            return
        
        marker_array = MarkerArray()
        marker_id = 0
        
        for obstacle_id, prediction in self.obstacle_predictions.items():
            if obstacle_id not in self.obstacles:
                continue
            
            obstacle = self.obstacles[obstacle_id]
            
            # Create prediction line
            if len(prediction.predicted_positions) > 1:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"prediction_{obstacle_id}"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                
                # Add points for prediction line
                for i, (x, y) in enumerate(prediction.predicted_positions):
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = 0.05
                    marker.points.append(point)
                
                # Color based on confidence
                confidence = prediction.confidence
                marker.color.r = 1.0 - confidence
                marker.color.g = confidence
                marker.color.b = 0.0
                marker.color.a = 0.6
                
                marker.scale.x = 0.1
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        if marker_array.markers:
            self.predictions_pub.publish(marker_array)
    
    def publish_occupancy_grid(self):
        """Publish occupancy grid for obstacle mapping"""
        if not self.mapping_enabled:
            return
        
        # Create occupancy grid
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size
        grid_msg.info.origin.position.x = -10.0
        grid_msg.info.origin.position.y = -10.0
        grid_msg.info.origin.position.z = 0.0
        
        # Initialize grid data
        grid_data = [0] * (self.grid_size * self.grid_size)
        
        # Mark obstacles in grid
        for obstacle in self.obstacles.values():
            grid_x = int((obstacle.x + 10.0) / self.grid_resolution)
            grid_y = int((obstacle.y + 10.0) / self.grid_resolution)
            
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                # Mark obstacle area
                radius_cells = int(obstacle.radius / self.grid_resolution)
                for dx in range(-radius_cells, radius_cells + 1):
                    for dy in range(-radius_cells, radius_cells + 1):
                        cell_x = grid_x + dx
                        cell_y = grid_y + dy
                        if (0 <= cell_x < self.grid_size and 
                            0 <= cell_y < self.grid_size):
                            distance = math.sqrt(dx*dx + dy*dy)
                            if distance <= radius_cells:
                                index = cell_y * self.grid_size + cell_x
                                # Higher occupancy for dangerous obstacles
                                occupancy = 100 if obstacle.obstacle_type == ObstacleType.DANGEROUS else 80
                                grid_data[index] = max(grid_data[index], occupancy)
        
        grid_msg.data = grid_data
        self.occupancy_grid_pub.publish(grid_msg)
    
    def publish_metrics(self):
        """Publish obstacle system metrics"""
        # Calculate overall metrics
        total_obstacles = len(self.obstacles)
        total_predictions = len(self.obstacle_predictions)
        
        # Calculate average avoidance success rate
        if self.total_interactions > 0:
            total_success = sum(obs.metrics.avoidance_success_rate * obs.metrics.interaction_count 
                              for obs in self.obstacles.values())
            self.avoidance_success_rate = total_success / self.total_interactions
        
        # Calculate average prediction accuracy (simplified)
        if total_predictions > 0:
            avg_confidence = sum(pred.confidence for pred in self.obstacle_predictions.values())
            self.average_prediction_accuracy = avg_confidence / total_predictions
        
        # Publish metrics
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            float(total_obstacles),
            float(total_predictions),
            self.avoidance_success_rate,
            self.average_prediction_accuracy,
            float(self.total_interactions),
            float(self.total_obstacles_created)
        ]
        
        self.obstacle_metrics_pub.publish(metrics_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = (
            f"Obstacles: {total_obstacles}, Predictions: {total_predictions}, "
            f"Success Rate: {self.avoidance_success_rate:.2f}, "
            f"Prediction Accuracy: {self.average_prediction_accuracy:.2f}"
        )
        self.obstacle_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    obstacle_system = AdvancedObstacleSystem()
    
    try:
        rclpy.spin(obstacle_system)
    except KeyboardInterrupt:
        obstacle_system.get_logger().info("🛑 Advanced Obstacle System stopped by user")
    finally:
        obstacle_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 