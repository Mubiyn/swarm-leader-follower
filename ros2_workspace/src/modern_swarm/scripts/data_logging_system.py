#!/usr/bin/env python3
"""
Data Logging and Analysis System for ROS2 Swarm

This module provides comprehensive data logging, analysis, and visualization capabilities:
- Real-time metrics collection from all swarm components
- Performance analysis and trend detection
- Data visualization with plots and charts
- Export capabilities for analysis
- Historical data storage and retrieval
- Automated reporting and alerts
"""

import numpy as np
import math
import time
import json
import csv
import os
from datetime import datetime, timedelta
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass, asdict
from collections import defaultdict, deque
from enum import Enum
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray, Bool, Int32
from geometry_msgs.msg import Point, Twist, Pose
from std_srvs.srv import SetBool
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io
import base64


class MetricType(Enum):
    """Types of metrics that can be logged"""
    POSITION = "position"
    VELOCITY = "velocity"
    FORMATION_ERROR = "formation_error"
    OBSTACLE_DISTANCE = "obstacle_distance"
    CONTROLLER_OUTPUT = "controller_output"
    PERFORMANCE = "performance"
    VISION_DETECTION = "vision_detection"
    SYSTEM_HEALTH = "system_health"
    ENERGY_CONSUMPTION = "energy_consumption"
    COMMUNICATION = "communication"


class LogLevel(Enum):
    """Log levels for different types of data"""
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4


@dataclass
class MetricData:
    """Structure for metric data points"""
    timestamp: float
    robot_id: str
    metric_type: MetricType
    value: Union[float, List[float], Dict[str, float]]
    metadata: Dict[str, Any] = None


@dataclass
class PerformanceMetrics:
    """Performance metrics structure"""
    timestamp: float
    formation_error: float
    obstacle_avoidance_effectiveness: float
    controller_response_time: float
    vision_accuracy: float
    system_latency: float
    energy_efficiency: float
    communication_reliability: float


@dataclass
class SystemEvent:
    """System event structure"""
    timestamp: float
    event_type: str
    severity: LogLevel
    description: str
    robot_id: Optional[str] = None
    data: Dict[str, Any] = None


class DataLogger(Node):
    """
    Comprehensive data logging and analysis system
    """
    
    def __init__(self):
        super().__init__('data_logger')
        
        # Declare parameters
        self.declare_parameter('log_directory', 'logs')
        self.declare_parameter('max_log_size_mb', 100.0)
        self.declare_parameter('log_retention_days', 7)
        self.declare_parameter('metrics_buffer_size', 10000)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('enable_export', True)
        
        # Initialize parameters
        self.log_directory = self.get_parameter('log_directory').value
        self.max_log_size_mb = self.get_parameter('max_log_size_mb').value
        self.log_retention_days = self.get_parameter('log_retention_days').value
        self.metrics_buffer_size = self.get_parameter('metrics_buffer_size').value
        self.update_rate = self.get_parameter('update_rate').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.enable_export = self.get_parameter('enable_export').value
        
        # Data storage
        self.metrics_buffer = deque(maxlen=self.metrics_buffer_size)
        self.performance_history = deque(maxlen=1000)
        self.system_events = deque(maxlen=1000)
        self.robot_positions = defaultdict(lambda: deque(maxlen=1000))
        self.formation_errors = deque(maxlen=1000)
        self.obstacle_distances = defaultdict(lambda: deque(maxlen=1000))
        
        # Analysis data
        self.analysis_results = {}
        self.trend_data = {}
        self.alert_conditions = {}
        
        # Threading
        self.data_queue = queue.Queue()
        self.visualization_data = {}
        self.lock = threading.Lock()
        
        # Setup logging directory
        self.setup_logging_directory()
        
        # Setup ROS2 interface
        self.setup_ros_interface()
        
        # Setup timers
        self.setup_timers()
        
        # Start background threads
        self.start_background_threads()
        
        self.get_logger().info("📊 Data Logging System initialized!")
        self.get_logger().info(f"📁 Log directory: {self.log_directory}")
        self.get_logger().info(f"📈 Visualization enabled: {self.enable_visualization}")
        self.get_logger().info(f"💾 Export enabled: {self.enable_export}")
    
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
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers, subscribers, and services"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers for data collection
        self.robot_poses_sub = self.create_subscription(
            Pose, '/swarm/robot_poses', self.robot_poses_callback, qos_profile
        )
        
        self.robot_velocities_sub = self.create_subscription(
            Twist, '/swarm/robot_velocities', self.robot_velocities_callback, qos_profile
        )
        
        self.formation_error_sub = self.create_subscription(
            Float32MultiArray, '/swarm/formation_error', self.formation_error_callback, qos_profile
        )
        
        self.obstacle_distances_sub = self.create_subscription(
            Float32MultiArray, '/swarm/obstacle_distances', self.obstacle_distances_callback, qos_profile
        )
        
        self.performance_metrics_sub = self.create_subscription(
            Float32MultiArray, '/swarm/performance_metrics', self.performance_metrics_callback, qos_profile
        )
        
        self.vision_detections_sub = self.create_subscription(
            String, '/swarm/vision_detections', self.vision_detections_callback, qos_profile
        )
        
        self.control_status_sub = self.create_subscription(
            String, '/swarm/control/status', self.control_status_callback, qos_profile
        )
        
        # Publishers for analysis results
        self.analysis_results_pub = self.create_publisher(
            String, '/swarm/analysis/results', qos_profile
        )
        
        self.visualization_data_pub = self.create_publisher(
            String, '/swarm/visualization/data', qos_profile
        )
        
        self.alert_pub = self.create_publisher(
            String, '/swarm/alerts', qos_profile
        )
        
        # Services
        self.export_data_srv = self.create_service(
            SetBool, '/swarm/logging/export_data', self.export_data_callback
        )
        
        self.generate_report_srv = self.create_service(
            SetBool, '/swarm/logging/generate_report', self.generate_report_callback
        )
        
        self.clear_logs_srv = self.create_service(
            SetBool, '/swarm/logging/clear_logs', self.clear_logs_callback
        )
    
    def setup_timers(self):
        """Setup ROS2 timers"""
        self.analysis_timer = self.create_timer(
            1.0 / self.update_rate, self.perform_analysis
        )
        
        self.visualization_timer = self.create_timer(2.0, self.update_visualization)
        
        self.cleanup_timer = self.create_timer(3600.0, self.cleanup_old_logs)  # Every hour
    
    def start_background_threads(self):
        """Start background processing threads"""
        self.data_processing_thread = threading.Thread(target=self.data_processing_worker, daemon=True)
        self.data_processing_thread.start()
        
        if self.enable_visualization:
            self.visualization_thread = threading.Thread(target=self.visualization_worker, daemon=True)
            self.visualization_thread.start()
    
    def robot_poses_callback(self, msg):
        """Callback for robot pose data"""
        try:
            # Extract robot ID from topic or message
            robot_id = "robot_1"  # This would come from the message or topic
            
            position = [msg.position.x, msg.position.y, msg.position.z]
            
            metric = MetricData(
                timestamp=time.time(),
                robot_id=robot_id,
                metric_type=MetricType.POSITION,
                value=position,
                metadata={'orientation': [msg.orientation.x, msg.orientation.y, 
                                        msg.orientation.z, msg.orientation.w]}
            )
            
            self.log_metric(metric)
            
            # Store in position history
            with self.lock:
                self.robot_positions[robot_id].append({
                    'timestamp': metric.timestamp,
                    'position': position
                })
                
        except Exception as e:
            self.get_logger().error(f"Error processing robot pose: {e}")
    
    def robot_velocities_callback(self, msg):
        """Callback for robot velocity data"""
        try:
            robot_id = "robot_1"  # This would come from the message or topic
            
            velocity = [msg.linear.x, msg.linear.y, msg.linear.z,
                       msg.angular.x, msg.angular.y, msg.angular.z]
            
            metric = MetricData(
                timestamp=time.time(),
                robot_id=robot_id,
                metric_type=MetricType.VELOCITY,
                value=velocity
            )
            
            self.log_metric(metric)
            
        except Exception as e:
            self.get_logger().error(f"Error processing robot velocity: {e}")
    
    def formation_error_callback(self, msg):
        """Callback for formation error data"""
        try:
            if len(msg.data) >= 2:
                formation_error = msg.data[0]
                robot_id = f"robot_{int(msg.data[1])}"
                
                metric = MetricData(
                    timestamp=time.time(),
                    robot_id=robot_id,
                    metric_type=MetricType.FORMATION_ERROR,
                    value=formation_error
                )
                
                self.log_metric(metric)
                
                # Store in formation error history
                with self.lock:
                    self.formation_errors.append({
                        'timestamp': metric.timestamp,
                        'error': formation_error,
                        'robot_id': robot_id
                    })
                
        except Exception as e:
            self.get_logger().error(f"Error processing formation error: {e}")
    
    def obstacle_distances_callback(self, msg):
        """Callback for obstacle distance data"""
        try:
            if len(msg.data) >= 2:
                distance = msg.data[0]
                robot_id = f"robot_{int(msg.data[1])}"
                
                metric = MetricData(
                    timestamp=time.time(),
                    robot_id=robot_id,
                    metric_type=MetricType.OBSTACLE_DISTANCE,
                    value=distance
                )
                
                self.log_metric(metric)
                
                # Store in obstacle distance history
                with self.lock:
                    self.obstacle_distances[robot_id].append({
                        'timestamp': metric.timestamp,
                        'distance': distance
                    })
                
        except Exception as e:
            self.get_logger().error(f"Error processing obstacle distance: {e}")
    
    def performance_metrics_callback(self, msg):
        """Callback for performance metrics data"""
        try:
            if len(msg.data) >= 8:
                metrics = PerformanceMetrics(
                    timestamp=time.time(),
                    formation_error=msg.data[0],
                    obstacle_avoidance_effectiveness=msg.data[1],
                    controller_response_time=msg.data[2],
                    vision_accuracy=msg.data[3],
                    system_latency=msg.data[4],
                    energy_efficiency=msg.data[5],
                    communication_reliability=msg.data[6]
                )
                
                with self.lock:
                    self.performance_history.append(metrics)
                
                # Log as metric
                metric = MetricData(
                    timestamp=metrics.timestamp,
                    robot_id="system",
                    metric_type=MetricType.PERFORMANCE,
                    value=asdict(metrics)
                )
                
                self.log_metric(metric)
                
        except Exception as e:
            self.get_logger().error(f"Error processing performance metrics: {e}")
    
    def vision_detections_callback(self, msg):
        """Callback for vision detection data"""
        try:
            data = json.loads(msg.data)
            
            metric = MetricData(
                timestamp=time.time(),
                robot_id=data.get('robot_id', 'unknown'),
                metric_type=MetricType.VISION_DETECTION,
                value=data.get('detections', []),
                metadata={'confidence': data.get('confidence', 0.0)}
            )
            
            self.log_metric(metric)
            
        except Exception as e:
            self.get_logger().error(f"Error processing vision detections: {e}")
    
    def control_status_callback(self, msg):
        """Callback for control system status"""
        try:
            data = json.loads(msg.data)
            
            # Log system health event
            event = SystemEvent(
                timestamp=time.time(),
                event_type="control_status_update",
                severity=LogLevel.INFO,
                description=f"Control mode: {data.get('mode', 'unknown')}",
                data=data
            )
            
            self.log_event(event)
            
        except Exception as e:
            self.get_logger().error(f"Error processing control status: {e}")
    
    def log_metric(self, metric: MetricData):
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
    
    def process_metric(self, metric: MetricData):
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
    
    def check_alert_conditions(self, metric: MetricData):
        """Check for alert conditions based on metrics"""
        try:
            if metric.metric_type == MetricType.FORMATION_ERROR:
                if metric.value > 2.0:  # Formation error threshold
                    alert = {
                        'timestamp': metric.timestamp,
                        'type': 'formation_error_high',
                        'severity': 'warning',
                        'robot_id': metric.robot_id,
                        'value': metric.value,
                        'threshold': 2.0
                    }
                    self.publish_alert(alert)
            
            elif metric.metric_type == MetricType.OBSTACLE_DISTANCE:
                if metric.value < 0.5:  # Obstacle too close
                    alert = {
                        'timestamp': metric.timestamp,
                        'type': 'obstacle_too_close',
                        'severity': 'critical',
                        'robot_id': metric.robot_id,
                        'value': metric.value,
                        'threshold': 0.5
                    }
                    self.publish_alert(alert)
            
            elif metric.metric_type == MetricType.PERFORMANCE:
                if isinstance(metric.value, dict):
                    formation_error = metric.value.get('formation_error', 0.0)
                    if formation_error > 3.0:
                        alert = {
                            'timestamp': metric.timestamp,
                            'type': 'performance_degraded',
                            'severity': 'warning',
                            'robot_id': metric.robot_id,
                            'value': formation_error,
                            'threshold': 3.0
                        }
                        self.publish_alert(alert)
                        
        except Exception as e:
            self.get_logger().error(f"Error checking alert conditions: {e}")
    
    def publish_alert(self, alert: Dict[str, Any]):
        """Publish an alert"""
        try:
            alert_msg = String()
            alert_msg.data = json.dumps(alert)
            self.alert_pub.publish(alert_msg)
            
            self.get_logger().warn(f"🚨 Alert: {alert['type']} - {alert['description']}")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing alert: {e}")
    
    def update_trend_analysis(self, metric: MetricData):
        """Update trend analysis for metrics"""
        try:
            metric_key = f"{metric.robot_id}_{metric.metric_type.value}"
            
            if metric_key not in self.trend_data:
                self.trend_data[metric_key] = {
                    'values': deque(maxlen=100),
                    'timestamps': deque(maxlen=100),
                    'mean': 0.0,
                    'std': 0.0,
                    'trend': 'stable'
                }
            
            trend = self.trend_data[metric_key]
            trend['values'].append(metric.value)
            trend['timestamps'].append(metric.timestamp)
            
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
                        'avg_formation_error': np.mean([m.formation_error for m in recent_metrics]),
                        'avg_obstacle_avoidance': np.mean([m.obstacle_avoidance_effectiveness for m in recent_metrics]),
                        'avg_controller_response': np.mean([m.controller_response_time for m in recent_metrics]),
                        'avg_vision_accuracy': np.mean([m.vision_accuracy for m in recent_metrics]),
                        'avg_system_latency': np.mean([m.system_latency for m in recent_metrics]),
                        'avg_energy_efficiency': np.mean([m.energy_efficiency for m in recent_metrics]),
                        'avg_communication_reliability': np.mean([m.communication_reliability for m in recent_metrics])
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
            analysis = {
                'timestamp': time.time(),
                'performance_stats': self.analysis_results.get('performance_stats', {}),
                'formation_error_stats': self.analysis_results.get('formation_error_stats', {}),
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
            
            return recommendations
            
        except Exception as e:
            self.get_logger().error(f"Error generating recommendations: {e}")
            return ["Error generating recommendations"]
    
    def publish_analysis_results(self):
        """Publish analysis results"""
        try:
            if 'comprehensive' in self.analysis_results:
                results_msg = String()
                results_msg.data = json.dumps(self.analysis_results['comprehensive'])
                self.analysis_results_pub.publish(results_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error publishing analysis results: {e}")
    
    def update_visualization(self):
        """Update visualization data"""
        if not self.enable_visualization:
            return
        
        try:
            # Generate visualization data
            viz_data = self.generate_visualization_data()
            
            # Publish visualization data
            viz_msg = String()
            viz_msg.data = json.dumps(viz_data)
            self.visualization_data_pub.publish(viz_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error updating visualization: {e}")
    
    def generate_visualization_data(self) -> Dict[str, Any]:
        """Generate data for visualization"""
        try:
            with self.lock:
                # Robot positions
                positions = {}
                for robot_id, pos_history in self.robot_positions.items():
                    if pos_history:
                        latest = list(pos_history)[-1]
                        positions[robot_id] = latest['position']
                
                # Formation errors over time
                formation_errors = []
                if self.formation_errors:
                    for error_data in list(self.formation_errors)[-100:]:
                        formation_errors.append({
                            'timestamp': error_data['timestamp'],
                            'error': error_data['error']
                        })
                
                # Performance metrics
                performance_data = []
                if self.performance_history:
                    for metric in list(self.performance_history)[-50:]:
                        performance_data.append({
                            'timestamp': metric.timestamp,
                            'formation_error': metric.formation_error,
                            'obstacle_avoidance': metric.obstacle_avoidance_effectiveness,
                            'vision_accuracy': metric.vision_accuracy
                        })
                
                return {
                    'robot_positions': positions,
                    'formation_errors': formation_errors,
                    'performance_data': performance_data,
                    'system_health': self.analysis_results.get('comprehensive', {}).get('system_health', {}),
                    'timestamp': time.time()
                }
                
        except Exception as e:
            self.get_logger().error(f"Error generating visualization data: {e}")
            return {}
    
    def visualization_worker(self):
        """Background worker for visualization generation"""
        while rclpy.ok() and self.enable_visualization:
            try:
                # Generate plots
                self.generate_performance_plots()
                time.sleep(5.0)  # Update every 5 seconds
                
            except Exception as e:
                self.get_logger().error(f"Error in visualization worker: {e}")
                time.sleep(1.0)
    
    def generate_performance_plots(self):
        """Generate performance plots"""
        try:
            with self.lock:
                if len(self.performance_history) < 10:
                    return
                
                # Create figure with subplots
                fig, axes = plt.subplots(2, 2, figsize=(12, 8))
                fig.suptitle('Swarm Performance Metrics', fontsize=16)
                
                # Get recent data
                recent_metrics = list(self.performance_history)[-100:]
                timestamps = [m.timestamp for m in recent_metrics]
                
                # Formation error over time
                formation_errors = [m.formation_error for m in recent_metrics]
                axes[0, 0].plot(timestamps, formation_errors, 'b-', linewidth=2)
                axes[0, 0].set_title('Formation Error Over Time')
                axes[0, 0].set_ylabel('Error (m)')
                axes[0, 0].grid(True)
                
                # Obstacle avoidance effectiveness
                avoidance = [m.obstacle_avoidance_effectiveness for m in recent_metrics]
                axes[0, 1].plot(timestamps, avoidance, 'g-', linewidth=2)
                axes[0, 1].set_title('Obstacle Avoidance Effectiveness')
                axes[0, 1].set_ylabel('Effectiveness')
                axes[0, 1].grid(True)
                
                # Vision accuracy
                vision = [m.vision_accuracy for m in recent_metrics]
                axes[1, 0].plot(timestamps, vision, 'r-', linewidth=2)
                axes[1, 0].set_title('Vision Accuracy')
                axes[1, 0].set_ylabel('Accuracy')
                axes[1, 0].grid(True)
                
                # System latency
                latency = [m.system_latency for m in recent_metrics]
                axes[1, 1].plot(timestamps, latency, 'm-', linewidth=2)
                axes[1, 1].set_title('System Latency')
                axes[1, 1].set_ylabel('Latency (ms)')
                axes[1, 1].grid(True)
                
                # Save plot
                plot_path = os.path.join(self.log_directory, 'visualizations', 
                                       f'performance_plot_{int(time.time())}.png')
                plt.tight_layout()
                plt.savefig(plot_path, dpi=150, bbox_inches='tight')
                plt.close()
                
                self.get_logger().info(f"📊 Generated performance plot: {plot_path}")
                
        except Exception as e:
            self.get_logger().error(f"Error generating performance plots: {e}")
    
    def save_metrics_to_file(self):
        """Save metrics to file"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.log_directory, 'metrics', f'metrics_{timestamp}.json')
            
            with self.lock:
                metrics_data = []
                for metric in list(self.metrics_buffer)[-100:]:  # Last 100 metrics
                    metrics_data.append({
                        'timestamp': metric.timestamp,
                        'robot_id': metric.robot_id,
                        'metric_type': metric.metric_type.value,
                        'value': metric.value,
                        'metadata': metric.metadata
                    })
            
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
    
    # Service callbacks
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
                            metric.timestamp,
                            metric.robot_id,
                            metric.metric_type.value,
                            str(metric.value),
                            json.dumps(metric.metadata) if metric.metadata else ''
                        ])
                
                # Export performance history
                perf_file = os.path.join(export_dir, 'performance_history.csv')
                with open(perf_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'formation_error', 'obstacle_avoidance', 
                                   'controller_response', 'vision_accuracy', 'system_latency',
                                   'energy_efficiency', 'communication_reliability'])
                    
                    for metric in self.performance_history:
                        writer.writerow([
                            metric.timestamp,
                            metric.formation_error,
                            metric.obstacle_avoidance_effectiveness,
                            metric.controller_response_time,
                            metric.vision_accuracy,
                            metric.system_latency,
                            metric.energy_efficiency,
                            metric.communication_reliability
                        ])
                
                # Export analysis results
                analysis_file = os.path.join(export_dir, 'analysis_results.json')
                with open(analysis_file, 'w') as f:
                    json.dump(self.analysis_results, f, indent=2)
                
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
                self.robot_positions.clear()
                self.formation_errors.clear()
                self.obstacle_distances.clear()
            
            self.analysis_results.clear()
            self.trend_data.clear()
            
            self.get_logger().info("🗑️ All logged data cleared")
            
        except Exception as e:
            self.get_logger().error(f"Error clearing logs: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    data_logger = DataLogger()
    
    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        data_logger.get_logger().info("🛑 Data Logger stopped by user")
    finally:
        data_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 