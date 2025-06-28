#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
import time

class MarkerTestNode(Node):
    def __init__(self):
        super().__init__('marker_test')
        
        # Create publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, '/test_markers', 10)
        
        # Create timer to publish markers
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.get_logger().info("Marker test node started")
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Create a simple red sphere at origin
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker_array.markers.append(marker)
        
        # Create a blue cube at (5, 0, 0)
        marker2 = Marker()
        marker2.header.frame_id = "map"
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.ns = "test"
        marker2.id = 1
        marker2.type = Marker.CUBE
        marker2.action = Marker.ADD
        
        marker2.pose.position.x = 5.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0
        marker2.pose.orientation.w = 1.0
        
        marker2.scale.x = 1.0
        marker2.scale.y = 1.0
        marker2.scale.z = 1.0
        
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        marker2.color.a = 1.0
        
        marker_array.markers.append(marker2)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 