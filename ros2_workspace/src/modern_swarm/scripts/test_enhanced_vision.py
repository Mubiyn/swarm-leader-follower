#!/usr/bin/env python3
"""
Test Enhanced Vision System

This script tests the enhanced vision system by:
1. Starting the enhanced swarm system with vision
2. Monitoring vision detection topics
3. Testing vision-based formation control
4. Demonstrating fallback mechanisms
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point
import time
import sys
import os

# Add the demos/python path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../demos/python'))


class EnhancedVisionTester(Node):
    """Test node for enhanced vision system"""
    
    def __init__(self):
        super().__init__('enhanced_vision_tester')
        
        # Setup QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers for vision topics
        self.vision_detections_sub = self.create_subscription(
            Float32MultiArray, '/swarm/vision/detections', self.detections_callback, qos_profile
        )
        
        self.vision_leader_pos_sub = self.create_subscription(
            Point, '/swarm/vision/leader_position', self.leader_position_callback, qos_profile
        )
        
        self.vision_metrics_sub = self.create_subscription(
            Float32MultiArray, '/swarm/vision/metrics', self.metrics_callback, qos_profile
        )
        
        self.vision_status_sub = self.create_subscription(
            String, '/swarm/vision/status', self.status_callback, qos_profile
        )
        
        self.swarm_status_sub = self.create_subscription(
            String, '/swarm/status', self.swarm_status_callback, qos_profile
        )
        
        # Test state
        self.detection_count = 0
        self.leader_detections = 0
        self.test_start_time = time.time()
        self.last_metrics = None
        self.last_status = None
        
        # Setup timer for test reporting
        self.test_timer = self.create_timer(5.0, self.report_test_status)
        
        self.get_logger().info("🧪 Enhanced Vision System Tester initialized!")
        self.get_logger().info("📊 Monitoring vision detection performance...")
    
    def detections_callback(self, msg: Float32MultiArray):
        """Callback for vision detections"""
        self.detection_count += 1
        
        # Parse detection data
        data = msg.data
        detections = []
        
        for i in range(0, len(data), 6):
            if i + 5 < len(data):
                robot_id = int(data[i])
                x = data[i + 1]
                y = data[i + 2]
                theta = data[i + 3]
                confidence = data[i + 4]
                area = data[i + 5]
                
                detections.append({
                    'robot_id': robot_id,
                    'x': x,
                    'y': y,
                    'theta': theta,
                    'confidence': confidence,
                    'area': area
                })
        
        if detections:
            self.get_logger().info(f"🎯 Detected {len(detections)} robots:")
            for det in detections:
                robot_name = "Leader" if det['robot_id'] == 0 else f"Follower_{det['robot_id']}"
                self.get_logger().info(f"   {robot_name}: ({det['x']:.2f}, {det['y']:.2f}), "
                                     f"confidence={det['confidence']:.1f}")
    
    def leader_position_callback(self, msg: Point):
        """Callback for detected leader position"""
        self.leader_detections += 1
        self.get_logger().info(f"👑 Leader detected at: ({msg.x:.2f}, {msg.y:.2f})")
    
    def metrics_callback(self, msg: Float32MultiArray):
        """Callback for vision metrics"""
        if len(msg.data) >= 6:
            self.last_metrics = {
                'detection_rate': msg.data[0],
                'avg_confidence': msg.data[1],
                'processing_time': msg.data[2],
                'false_positives': msg.data[3],
                'missed_detections': msg.data[4],
                'total_detections': msg.data[5]
            }
    
    def status_callback(self, msg: String):
        """Callback for vision status"""
        self.last_status = msg.data
        self.get_logger().info(f"📊 Vision Status: {msg.data}")
    
    def swarm_status_callback(self, msg: String):
        """Callback for swarm system status"""
        # Extract vision detection info from swarm status
        if "Vision Detection: ON" in msg.data:
            if "DETECTED" in msg.data:
                self.get_logger().info("✅ Vision-based formation control is ACTIVE")
            else:
                self.get_logger().info("⚠️ Vision detection enabled but no leader detected")
    
    def report_test_status(self):
        """Report test status every 5 seconds"""
        runtime = time.time() - self.test_start_time
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🧪 ENHANCED VISION SYSTEM TEST REPORT")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"⏱️  Runtime: {runtime:.1f}s")
        self.get_logger().info(f"🎯 Total detections: {self.detection_count}")
        self.get_logger().info(f"👑 Leader detections: {self.leader_detections}")
        
        if self.last_metrics:
            self.get_logger().info("📊 Vision Performance Metrics:")
            self.get_logger().info(f"   Detection Rate: {self.last_metrics['detection_rate']:.3f}")
            self.get_logger().info(f"   Avg Confidence: {self.last_metrics['avg_confidence']:.3f}")
            self.get_logger().info(f"   Processing Time: {self.last_metrics['processing_time']:.3f}s")
            self.get_logger().info(f"   Total Detections: {self.last_metrics['total_detections']}")
            self.get_logger().info(f"   False Positives: {self.last_metrics['false_positives']}")
            self.get_logger().info(f"   Missed Detections: {self.last_metrics['missed_detections']}")
        
        if self.last_status:
            self.get_logger().info(f"📋 Current Status: {self.last_status}")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("💡 Test Instructions:")
        self.get_logger().info("   1. Use RViz to visualize robot positions and detections")
        self.get_logger().info("   2. Check camera view for robot detection visualization")
        self.get_logger().info("   3. Toggle vision detection: ros2 service call /swarm/toggle_vision_detection std_srvs/srv/SetBool '{data: true}'")
        self.get_logger().info("   4. Switch formations: ros2 service call /swarm/set_formation std_srvs/srv/SetBool '{data: true}'")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    
    tester = EnhancedVisionTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("🛑 Enhanced Vision Tester stopped by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 