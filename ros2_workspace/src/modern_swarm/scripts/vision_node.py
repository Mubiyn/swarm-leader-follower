#!/usr/bin/env python3

"""
Modern ROS2 Vision Node for Swarm Leader-Follower Navigation

This node replaces AR tag detection with YOLO-based robot detection
and provides enhanced computer vision capabilities for swarm robotics.

Migration from ROS1 vision.py:
- ROS1 → ROS2 API conversion
- AR tag detection → YOLO robot detection  
- Enhanced distance estimation using depth information
- Multi-robot tracking capabilities
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32, Header, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import torch
from ultralytics import YOLO
import json
from typing import List, Dict, Optional, Tuple


class ModernVisionNode(Node):
    """
    Modern ROS2 Vision Node for Robot Detection and Tracking
    
    Replaces the original AngleFinder class with:
    - YOLO-based robot detection instead of AR tags
    - Multi-robot tracking
    - Enhanced distance estimation
    - ROS2 native implementation
    """

    def __init__(self):
        super().__init__('modern_vision_node')
        
        # Declare parameters
        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('robot_width', 0.4)  # meters (equivalent to tag_width)
        self.declare_parameter('max_detection_range', 10.0)  # meters
        self.declare_parameter('simulation_mode', True)  # Add simulation mode
        
        # Get parameters
        self.robot_ns = self.get_parameter('robot_namespace').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.robot_width = self.get_parameter('robot_width').value
        self.max_range = self.get_parameter('max_detection_range').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # Initialize computer vision components
        self.bridge = CvBridge()
        self.cv_image = None
        self.camera_info = None
        
        # Camera parameters (from original vision.py)
        self.h_field_of_view = 1.3962634  # radians
        self.camera_width = 640  # Default width for simulation
        self.focal_length = None
        
        # YOLO model initialization
        if not self.simulation_mode:
            try:
                model_path = self.get_parameter('yolo_model_path').value
                self.yolo_model = YOLO(model_path)
                self.get_logger().info(f"YOLO model loaded: {model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.yolo_model = None
        else:
            self.yolo_model = None
            self.get_logger().info("Running in SIMULATION MODE - generating fake detections")
        
        # Robot detection tracking
        self.detected_robots = {}
        self.leader_robot_id = None
        
        # Simulation counter for fake detections
        self.sim_counter = 0
        
        # ROS2 Subscribers (only if not in simulation mode)
        if not self.simulation_mode:
            self.image_sub = self.create_subscription(
                Image,
                f'/{self.robot_ns}/camera/image_raw',
                self.process_image,
                10
            )
            
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                f'/{self.robot_ns}/camera/camera_info',
                self.process_camera_info,
                10
            )
        
        # ROS2 Publishers  
        self.angle_pub = self.create_publisher(
            Float32,
            f'/{self.robot_ns}/angle_to_leader',
            10
        )
        
        self.leader_position_pub = self.create_publisher(
            PointStamped,
            f'/{self.robot_ns}/leader_position',
            10
        )
        
        self.detected_robots_pub = self.create_publisher(
            String,
            f'/{self.robot_ns}/detected_robots',
            10
        )
        
        # Timer for main processing loop
        self.timer = self.create_timer(0.2, self.run_detection)  # 5 Hz
        
        self.get_logger().info(f"Modern Vision Node initialized for {self.robot_ns}")
        if self.simulation_mode:
            self.get_logger().info("⚡ SIMULATION MODE: Will generate fake robot detections")

    def process_image(self, msg: Image) -> None:
        """Process incoming camera images"""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def process_camera_info(self, msg: CameraInfo) -> None:
        """Process camera calibration information"""
        self.camera_info = msg
        self.camera_width = msg.width
        self.focal_length = self.get_focal_length(self.camera_width, self.h_field_of_view)

    def get_focal_length(self, width: int, fov: float) -> float:
        """Calculate focal length from camera parameters"""
        return (width / 2) / (math.tan(fov / 2))

    def detect_robots_yolo(self, image: np.ndarray) -> List[Dict]:
        """
        Use YOLO to detect robots in the image
        
        Returns list of detections with format:
        [{'bbox': [x1, y1, x2, y2], 'confidence': float, 'class': str}]
        """
        if self.yolo_model is None:
            return []
        
        try:
            # Run YOLO detection
            results = self.yolo_model(image, conf=self.detection_confidence)
            
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.yolo_model.names[class_id]
                        
                        # Filter for relevant objects (robots, people, etc.)
                        if class_name in ['person', 'robot', 'car', 'bicycle']:
                            detections.append({
                                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                'confidence': float(confidence),
                                'class': class_name,
                                'center': [int((x1 + x2) / 2), int((y1 + y2) / 2)]
                            })
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f"YOLO detection failed: {e}")
            return []

    def estimate_distance(self, bbox: List[int]) -> float:
        """
        Estimate distance to detected robot using bounding box width
        (Replaces get_distance_to_camera from original)
        """
        x1, y1, x2, y2 = bbox
        perceived_width = x2 - x1
        
        if perceived_width > 0 and self.focal_length is not None:
            # Similar calculation to original vision.py
            distance = (self.robot_width * self.focal_length) / perceived_width
            return min(distance, self.max_range)  # Cap at max range
        
        return float('inf')

    def calculate_angle_to_robot(self, center_x: int, center_y: int) -> float:
        """
        Calculate angle to robot from camera center
        (Same logic as original get_angle method)
        """
        if self.camera_width is None:
            return 0.0
        
        return ((center_x - float(self.camera_width / 2)) / self.camera_width) * \
               (math.degrees(self.h_field_of_view))

    def select_leader_robot(self, detections: List[Dict]) -> Optional[Dict]:
        """
        Select the leader robot from detected robots
        Strategy: Closest robot with high confidence
        """
        if not detections:
            return None
        
        # Filter high-confidence detections
        good_detections = [d for d in detections if d['confidence'] > 0.7]
        if not good_detections:
            good_detections = detections
        
        # Select closest robot as leader
        leader = min(good_detections, 
                    key=lambda d: self.estimate_distance(d['bbox']))
        
        return leader

    def visualize_detections(self, image: np.ndarray, detections: List[Dict], 
                           leader: Optional[Dict]) -> np.ndarray:
        """Draw detection results on image"""
        vis_image = image.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class']
            
            # Different color for leader
            if leader and detection == leader:
                color = (0, 255, 0)  # Green for leader
                thickness = 3
                label = f"LEADER {class_name}: {confidence:.2f}"
            else:
                color = (255, 0, 0)  # Blue for other robots
                thickness = 2
                label = f"{class_name}: {confidence:.2f}"
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, thickness)
            
            # Draw label
            cv2.putText(vis_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw center point
            center_x, center_y = detection['center']
            cv2.circle(vis_image, (center_x, center_y), 5, color, -1)
        
        return vis_image

    def generate_fake_detections(self) -> List[Dict]:
        """
        Generate fake robot detections for simulation mode
        Simulates a leader robot moving in a circle
        """
        self.sim_counter += 1
        
        # Simulate a robot moving in a circle
        import math
        angle = (self.sim_counter * 0.1) % (2 * math.pi)  # Slow circular movement
        
        # Generate fake detection
        center_x = 320 + int(100 * math.cos(angle))  # Center ± 100 pixels
        center_y = 240 + int(50 * math.sin(angle))   # Center ± 50 pixels
        
        detection = {
            'bbox': [center_x - 50, center_y - 50, center_x + 50, center_y + 50],
            'confidence': 0.85,
            'class': 'person',  # Simulate detecting a person as robot
            'center': [center_x, center_y]
        }
        
        return [detection]

    def run_detection(self) -> None:
        """Main detection loop (replaces original run method)"""
        if self.simulation_mode:
            # Generate fake detections in simulation mode
            detections = self.generate_fake_detections()
            self.focal_length = self.get_focal_length(self.camera_width, self.h_field_of_view)
        else:
            # Real detection mode
            if self.cv_image is None or self.camera_width is None:
                return
            
            # Detect robots using YOLO
            detections = self.detect_robots_yolo(self.cv_image)
        
        if detections:
            # Select leader robot
            leader = self.select_leader_robot(detections)
            
            if leader:
                # Calculate angle to leader
                center_x, center_y = leader['center']
                angle = self.calculate_angle_to_robot(center_x, center_y)
                
                # Publish angle to leader
                angle_msg = Float32()
                angle_msg.data = angle
                self.angle_pub.publish(angle_msg)
                
                # Estimate distance and publish leader position
                distance = self.estimate_distance(leader['bbox'])
                
                position_msg = PointStamped()
                position_msg.header = Header()
                position_msg.header.stamp = self.get_clock().now().to_msg()
                position_msg.header.frame_id = f"{self.robot_ns}/camera_link"
                
                # Convert to 3D position (simplified)
                position_msg.point.x = distance * math.cos(math.radians(angle))
                position_msg.point.y = distance * math.sin(math.radians(angle))
                position_msg.point.z = 0.0
                
                self.leader_position_pub.publish(position_msg)
                
                mode_str = "SIMULATION" if self.simulation_mode else "REAL"
                self.get_logger().info(
                    f"{mode_str} - Leader detected: angle={angle:.1f}°, distance={distance:.2f}m"
                )
            
            # Publish all detected robots info
            robots_info = {
                'timestamp': self.get_clock().now().to_msg(),
                'count': len(detections),
                'robots': detections,
                'simulation_mode': self.simulation_mode
            }
            
            robots_msg = String()
            robots_msg.data = json.dumps(robots_info, default=str)
            self.detected_robots_pub.publish(robots_msg)
        
        # Only show visualization if not in simulation mode
        if not self.simulation_mode:
            # Visualize results (for debugging)
            if detections and self.cv_image is not None:
                leader = self.select_leader_robot(detections)
                vis_image = self.visualize_detections(self.cv_image, detections, leader)
                cv2.imshow(f'{self.robot_ns} Modern Vision', vis_image)
            elif self.cv_image is not None:
                cv2.imshow(f'{self.robot_ns} Modern Vision', self.cv_image)
            
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ModernVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 