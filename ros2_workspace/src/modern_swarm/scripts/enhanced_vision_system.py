#!/usr/bin/env python3
"""
Enhanced Vision System for ROS2 Swarm

This module provides advanced computer vision capabilities for the multi-robot
leader-follower swarm system, including:

- Real leader detection using camera data
- Multi-robot detection and tracking
- Vision-based formation control
- Detection confidence scoring
- Fallback mechanisms for robust operation
- Camera calibration and coordinate transformation
"""

import numpy as np
import cv2
import math
import time
from typing import List, Dict, Tuple, Optional, NamedTuple
from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped


class DetectionConfidence(Enum):
    """Detection confidence levels"""
    HIGH = 3
    MEDIUM = 2
    LOW = 1
    NONE = 0


@dataclass
class RobotDetection:
    """Represents a detected robot with confidence and metadata"""
    robot_id: int
    x: float
    y: float
    theta: float
    color: str
    confidence: DetectionConfidence
    pixel_position: Tuple[int, int]
    contour_area: float
    detection_time: float


@dataclass
class VisionMetrics:
    """Vision system performance metrics"""
    detection_rate: float
    average_confidence: float
    processing_time: float
    false_positives: int
    missed_detections: int
    total_detections: int


class EnhancedVisionSystem(Node):
    """
    Enhanced computer vision system for multi-robot detection and tracking
    """
    
    def __init__(self):
        super().__init__('enhanced_vision_system')
        
        # Declare parameters
        self.declare_parameter('camera_resolution', [640, 480])
        self.declare_parameter('camera_fov', [-10.0, 10.0, -10.0, 10.0])
        self.declare_parameter('detection_confidence_threshold', 0.6)
        self.declare_parameter('detection_history_length', 10)
        self.declare_parameter('min_contour_area', 100)
        self.declare_parameter('max_contour_area', 5000)
        self.declare_parameter('color_detection_enabled', True)
        self.declare_parameter('motion_detection_enabled', True)
        self.declare_parameter('vision_update_rate', 30.0)
        
        # Initialize vision parameters
        self.resolution = tuple(self.get_parameter('camera_resolution').value)
        self.fov = tuple(self.get_parameter('camera_fov').value)
        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        self.history_length = self.get_parameter('detection_history_length').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.max_contour_area = self.get_parameter('max_contour_area').value
        self.color_detection_enabled = self.get_parameter('color_detection_enabled').value
        self.motion_detection_enabled = self.get_parameter('motion_detection_enabled').value
        
        # Color ranges for robot detection (HSV)
        self.color_ranges = {
            'red': {
                'lower': np.array([0, 100, 100]), 
                'upper': np.array([10, 255, 255]),
                'lower2': np.array([170, 100, 100]),  # Red wraps around HSV
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 100, 100]), 
                'upper': np.array([130, 255, 255])
            },
            'green': {
                'lower': np.array([40, 100, 100]), 
                'upper': np.array([80, 255, 255])
            },
            'orange': {
                'lower': np.array([10, 100, 100]), 
                'upper': np.array([25, 255, 255])
            }
        }
        
        # Robot color mapping
        self.robot_colors = {
            0: 'red',    # Leader
            1: 'blue',   # Follower 1
            2: 'green',  # Follower 2
            3: 'orange'  # Follower 3
        }
        
        # Detection state
        self.detection_history = {robot_id: [] for robot_id in self.robot_colors.keys()}
        self.last_detections = {}
        self.vision_metrics = VisionMetrics(0.0, 0.0, 0.0, 0, 0, 0)
        self.frame_count = 0
        self.detection_count = 0
        
        # Camera calibration
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.calibration_enabled = False
        
        # Motion detection
        self.previous_frame = None
        self.motion_threshold = 25
        
        # CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Setup ROS2 interface
        self.setup_ros_interface()
        
        # Setup timers
        self.setup_timers()
        
        self.get_logger().info("🎥 Enhanced Vision System initialized!")
        self.get_logger().info(f"📊 Detection confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"🎨 Color detection: {'ON' if self.color_detection_enabled else 'OFF'}")
        self.get_logger().info(f"🏃 Motion detection: {'ON' if self.motion_detection_enabled else 'OFF'}")
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers, subscribers, and services"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/swarm/camera/image', self.camera_callback, qos_profile
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(
            Float32MultiArray, '/swarm/vision/detections', qos_profile
        )
        
        self.leader_position_pub = self.create_publisher(
            Point, '/swarm/vision/leader_position', qos_profile
        )
        
        self.vision_metrics_pub = self.create_publisher(
            Float32MultiArray, '/swarm/vision/metrics', qos_profile
        )
        
        self.detection_markers_pub = self.create_publisher(
            MarkerArray, '/swarm/vision/detection_markers', qos_profile
        )
        
        self.vision_status_pub = self.create_publisher(
            String, '/swarm/vision/status', qos_profile
        )
    
    def setup_timers(self):
        """Setup ROS2 timers"""
        self.vision_timer = self.create_timer(
            1.0 / self.get_parameter('vision_update_rate').value,
            self.vision_update_callback
        )
        
        self.metrics_timer = self.create_timer(1.0, self.publish_metrics)
    
    def camera_callback(self, msg: Image):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process the image for robot detection
            detections = self.detect_robots(cv_image)
            
            # Update detection history
            self.update_detection_history(detections)
            
            # Store detections for publishing
            self.last_detections = {det.robot_id: det for det in detections}
            
            # Update metrics
            self.update_vision_metrics(len(detections))
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")
    
    def detect_robots(self, image: np.ndarray) -> List[RobotDetection]:
        """Detect all robots in the image using multiple detection methods"""
        detections = []
        current_time = time.time()
        
        # Method 1: Color-based detection
        if self.color_detection_enabled:
            color_detections = self.color_based_detection(image)
            detections.extend(color_detections)
        
        # Method 2: Motion-based detection (if enabled and previous frame available)
        if self.motion_detection_enabled and self.previous_frame is not None:
            motion_detections = self.motion_based_detection(image)
            detections.extend(motion_detections)
        
        # Method 3: Contour-based detection (fallback)
        if not detections:
            contour_detections = self.contour_based_detection(image)
            detections.extend(contour_detections)
        
        # Filter and validate detections
        valid_detections = self.filter_detections(detections)
        
        # Update previous frame for motion detection
        self.previous_frame = image.copy()
        
        return valid_detections
    
    def color_based_detection(self, image: np.ndarray) -> List[RobotDetection]:
        """Detect robots using color-based segmentation"""
        detections = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        current_time = time.time()
        
        for robot_id, target_color in self.robot_colors.items():
            if target_color not in self.color_ranges:
                continue
            
            color_range = self.color_ranges[target_color]
            
            # Create mask for target color
            if target_color == 'red':
                # Red wraps around HSV, so we need two masks
                mask1 = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
                mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour for this color
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if self.min_contour_area <= area <= self.max_contour_area:
                    # Get centroid
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        pixel_x = int(M["m10"] / M["m00"])
                        pixel_y = int(M["m01"] / M["m00"])
                        
                        # Convert to world coordinates
                        world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
                        
                        # Estimate orientation from contour
                        theta = self.estimate_orientation(largest_contour, pixel_x, pixel_y)
                        
                        # Calculate confidence based on area and color match
                        confidence = self.calculate_color_confidence(area, target_color, mask, pixel_x, pixel_y)
                        
                        detection = RobotDetection(
                            robot_id=robot_id,
                            x=world_x,
                            y=world_y,
                            theta=theta,
                            color=target_color,
                            confidence=confidence,
                            pixel_position=(pixel_x, pixel_y),
                            contour_area=area,
                            detection_time=current_time
                        )
                        detections.append(detection)
        
        return detections
    
    def motion_based_detection(self, image: np.ndarray) -> List[RobotDetection]:
        """Detect robots using motion detection"""
        detections = []
        current_time = time.time()
        
        # Convert to grayscale
        gray_current = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_previous = cv2.cvtColor(self.previous_frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate frame difference
        frame_diff = cv2.absdiff(gray_current, gray_previous)
        
        # Apply threshold
        _, thresh = cv2.threshold(frame_diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours in motion areas
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_contour_area <= area <= self.max_contour_area:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    pixel_x = int(M["m10"] / M["m00"])
                    pixel_y = int(M["m01"] / M["m00"])
                    
                    # Convert to world coordinates
                    world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
                    
                    # Estimate orientation
                    theta = self.estimate_orientation(contour, pixel_x, pixel_y)
                    
                    # For motion detection, we don't know the robot ID, so assign based on position
                    robot_id = self.assign_robot_id_by_position(world_x, world_y)
                    color = self.robot_colors.get(robot_id, 'unknown')
                    
                    detection = RobotDetection(
                        robot_id=robot_id,
                        x=world_x,
                        y=world_y,
                        theta=theta,
                        color=color,
                        confidence=DetectionConfidence.MEDIUM,
                        pixel_position=(pixel_x, pixel_y),
                        contour_area=area,
                        detection_time=current_time
                    )
                    detections.append(detection)
        
        return detections
    
    def contour_based_detection(self, image: np.ndarray) -> List[RobotDetection]:
        """Fallback detection using general contour analysis"""
        detections = []
        current_time = time.time()
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Find edges
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_contour_area <= area <= self.max_contour_area:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    pixel_x = int(M["m10"] / M["m00"])
                    pixel_y = int(M["m01"] / M["m00"])
                    
                    # Convert to world coordinates
                    world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
                    
                    # Estimate orientation
                    theta = self.estimate_orientation(contour, pixel_x, pixel_y)
                    
                    # Assign robot ID based on position
                    robot_id = self.assign_robot_id_by_position(world_x, world_y)
                    color = self.robot_colors.get(robot_id, 'unknown')
                    
                    detection = RobotDetection(
                        robot_id=robot_id,
                        x=world_x,
                        y=world_y,
                        theta=theta,
                        color=color,
                        confidence=DetectionConfidence.LOW,
                        pixel_position=(pixel_x, pixel_y),
                        contour_area=area,
                        detection_time=current_time
                    )
                    detections.append(detection)
        
        return detections
    
    def filter_detections(self, detections: List[RobotDetection]) -> List[RobotDetection]:
        """Filter and validate detections"""
        filtered = []
        
        # Group detections by robot ID
        robot_detections = {}
        for det in detections:
            if det.robot_id not in robot_detections:
                robot_detections[det.robot_id] = []
            robot_detections[det.robot_id].append(det)
        
        # For each robot, select the best detection
        for robot_id, dets in robot_detections.items():
            if dets:
                # Sort by confidence and area
                best_detection = max(dets, key=lambda d: (d.confidence.value, d.contour_area))
                
                # Apply confidence threshold
                if best_detection.confidence.value >= self.confidence_threshold:
                    filtered.append(best_detection)
        
        return filtered
    
    def update_detection_history(self, detections: List[RobotDetection]):
        """Update detection history for each robot"""
        current_time = time.time()
        
        for detection in detections:
            robot_id = detection.robot_id
            if robot_id in self.detection_history:
                self.detection_history[robot_id].append(detection)
                
                # Limit history length
                if len(self.detection_history[robot_id]) > self.history_length:
                    self.detection_history[robot_id].pop(0)
    
    def get_smoothed_position(self, robot_id: int) -> Optional[Tuple[float, float]]:
        """Get smoothed position for a robot based on detection history"""
        if robot_id not in self.detection_history or not self.detection_history[robot_id]:
            return None
        
        # Calculate weighted average based on confidence and recency
        total_weight = 0.0
        weighted_x = 0.0
        weighted_y = 0.0
        current_time = time.time()
        
        for detection in self.detection_history[robot_id]:
            # Weight based on confidence and time
            time_weight = max(0, 1.0 - (current_time - detection.detection_time))
            confidence_weight = detection.confidence.value / 3.0
            weight = time_weight * confidence_weight
            
            weighted_x += detection.x * weight
            weighted_y += detection.y * weight
            total_weight += weight
        
        if total_weight > 0:
            return weighted_x / total_weight, weighted_y / total_weight
        
        return None
    
    def get_leader_position(self) -> Optional[Tuple[float, float]]:
        """Get the detected leader position"""
        return self.get_smoothed_position(0)  # Leader has robot_id 0
    
    def get_all_robot_positions(self) -> Dict[int, Tuple[float, float]]:
        """Get positions of all detected robots"""
        positions = {}
        for robot_id in self.robot_colors.keys():
            pos = self.get_smoothed_position(robot_id)
            if pos:
                positions[robot_id] = pos
        return positions
    
    def pixel_to_world(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates"""
        cam_x = pixel_x / self.resolution[0]
        cam_y = 1 - (pixel_y / self.resolution[1])  # Flip Y axis
        
        world_x = cam_x * (self.fov[1] - self.fov[0]) + self.fov[0]
        world_y = cam_y * (self.fov[3] - self.fov[2]) + self.fov[2]
        
        return world_x, world_y
    
    def world_to_pixel(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to pixel coordinates"""
        cam_x = (world_x - self.fov[0]) / (self.fov[1] - self.fov[0])
        cam_y = (world_y - self.fov[2]) / (self.fov[3] - self.fov[2])
        
        pixel_x = int(cam_x * self.resolution[0])
        pixel_y = int((1 - cam_y) * self.resolution[1])  # Flip Y axis
        
        return pixel_x, pixel_y
    
    def estimate_orientation(self, contour: np.ndarray, center_x: int, center_y: int) -> float:
        """Estimate robot orientation from contour"""
        try:
            # Fit ellipse to contour
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                angle = ellipse[2]  # Angle in degrees
                return math.radians(angle)
            
            # Fallback: use contour moments
            M = cv2.moments(contour)
            if M["mu20"] != M["mu02"]:
                angle = 0.5 * math.atan2(2 * M["mu11"], M["mu20"] - M["mu02"])
                return angle
            
        except Exception:
            pass
        
        return 0.0
    
    def calculate_color_confidence(self, area: float, target_color: str, mask: np.ndarray, 
                                 center_x: int, center_y: int) -> DetectionConfidence:
        """Calculate detection confidence based on color match and area"""
        # Check area consistency
        expected_area = 700  # Expected robot area in pixels
        area_confidence = 1.0 - abs(area - expected_area) / expected_area
        area_confidence = max(0.0, min(1.0, area_confidence))
        
        # Check color consistency in the region
        region_size = 20
        x1 = max(0, center_x - region_size)
        y1 = max(0, center_y - region_size)
        x2 = min(mask.shape[1], center_x + region_size)
        y2 = min(mask.shape[0], center_y + region_size)
        
        region = mask[y1:y2, x1:x2]
        color_ratio = np.sum(region > 0) / region.size if region.size > 0 else 0.0
        
        # Combined confidence
        confidence = (area_confidence + color_ratio) / 2.0
        
        if confidence > 0.8:
            return DetectionConfidence.HIGH
        elif confidence > 0.6:
            return DetectionConfidence.MEDIUM
        elif confidence > 0.4:
            return DetectionConfidence.LOW
        else:
            return DetectionConfidence.NONE
    
    def assign_robot_id_by_position(self, world_x: float, world_y: float) -> int:
        """Assign robot ID based on position (fallback method)"""
        # Simple heuristic: leader is usually near origin, followers are behind
        distance_from_origin = math.sqrt(world_x**2 + world_y**2)
        
        if distance_from_origin < 3.0:
            return 0  # Leader
        elif world_x < -2.0:
            if world_y < 0:
                return 1  # Follower 1 (left)
            else:
                return 2  # Follower 2 (right)
        else:
            return 3  # Follower 3 (back)
    
    def update_vision_metrics(self, num_detections: int):
        """Update vision system performance metrics"""
        self.frame_count += 1
        self.detection_count += num_detections
        
        # Calculate detection rate
        if self.frame_count > 0:
            self.vision_metrics.detection_rate = self.detection_count / self.frame_count
        
        # Calculate average confidence
        if self.last_detections:
            avg_confidence = sum(det.confidence.value for det in self.last_detections.values()) / len(self.last_detections)
            self.vision_metrics.average_confidence = avg_confidence
    
    def vision_update_callback(self):
        """Periodic vision system update"""
        # Publish detections
        self.publish_detections()
        
        # Publish leader position
        self.publish_leader_position()
        
        # Publish detection markers
        self.publish_detection_markers()
        
        # Publish vision status
        self.publish_vision_status()
    
    def publish_detections(self):
        """Publish all robot detections"""
        if not self.last_detections:
            return
        
        detections_msg = Float32MultiArray()
        detections_msg.data = []
        
        for robot_id, detection in self.last_detections.items():
            detections_msg.data.extend([
                float(robot_id),
                detection.x,
                detection.y,
                detection.theta,
                float(detection.confidence.value),
                detection.contour_area
            ])
        
        self.detections_pub.publish(detections_msg)
    
    def publish_leader_position(self):
        """Publish detected leader position"""
        leader_pos = self.get_leader_position()
        if leader_pos:
            leader_msg = Point()
            leader_msg.x = leader_pos[0]
            leader_msg.y = leader_pos[1]
            leader_msg.z = 0.0
            self.leader_position_pub.publish(leader_msg)
    
    def publish_detection_markers(self):
        """Publish detection markers for RViz visualization"""
        marker_array = MarkerArray()
        marker_id = 0
        
        for robot_id, detection in self.last_detections.items():
            # Create detection marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"vision_detection_{robot_id}"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = detection.x
            marker.pose.position.y = detection.y
            marker.pose.position.z = 0.1
            
            # Orientation
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(detection.theta / 2.0)
            marker.pose.orientation.w = math.cos(detection.theta / 2.0)
            
            # Scale
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            
            # Color based on confidence
            if detection.confidence == DetectionConfidence.HIGH:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif detection.confidence == DetectionConfidence.MEDIUM:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            marker.color.a = 0.7
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        if marker_array.markers:
            self.detection_markers_pub.publish(marker_array)
    
    def publish_vision_status(self):
        """Publish vision system status"""
        status_msg = String()
        
        if self.last_detections:
            detected_robots = list(self.last_detections.keys())
            avg_confidence = sum(det.confidence.value for det in self.last_detections.values()) / len(self.last_detections)
            status_msg.data = f"Active: {len(detected_robots)} robots detected, avg confidence: {avg_confidence:.2f}"
        else:
            status_msg.data = "No robots detected"
        
        self.vision_status_pub.publish(status_msg)
    
    def publish_metrics(self):
        """Publish vision system metrics"""
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            self.vision_metrics.detection_rate,
            self.vision_metrics.average_confidence,
            self.vision_metrics.processing_time,
            float(self.vision_metrics.false_positives),
            float(self.vision_metrics.missed_detections),
            float(self.vision_metrics.total_detections)
        ]
        
        self.vision_metrics_pub.publish(metrics_msg)


def main(args=None):
    rclpy.init(args=args)
    
    vision_system = EnhancedVisionSystem()
    
    try:
        rclpy.spin(vision_system)
    except KeyboardInterrupt:
        vision_system.get_logger().info("🛑 Enhanced Vision System stopped by user")
    finally:
        vision_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 