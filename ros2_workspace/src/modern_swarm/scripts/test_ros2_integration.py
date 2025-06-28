#!/usr/bin/env python3
"""
Test script for ROS2 integration
Tests all the services and topics of the unified swarm system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
import time


class SwarmTester(Node):
    """Test node for the unified swarm system"""
    
    def __init__(self):
        super().__init__('swarm_tester')
        
        # Service clients
        self.set_formation_client = self.create_client(String, '/swarm/set_formation')
        self.toggle_vision_client = self.create_client(Bool, '/swarm/toggle_vision')
        self.toggle_obstacles_client = self.create_client(Bool, '/swarm/toggle_obstacles')
        self.add_obstacle_client = self.create_client(Float32MultiArray, '/swarm/add_obstacle')
        self.set_controller_client = self.create_client(String, '/swarm/set_controller')
        
        # Subscribers for testing
        self.status_sub = self.create_subscription(
            String, '/swarm/status', self.status_callback, 10
        )
        self.leader_pose_sub = self.create_subscription(
            PoseStamped, '/swarm/leader/pose', self.leader_pose_callback, 10
        )
        self.robot_markers_sub = self.create_subscription(
            MarkerArray, '/swarm/robot_markers', self.robot_markers_callback, 10
        )
        self.obstacles_sub = self.create_subscription(
            MarkerArray, '/swarm/obstacles', self.obstacles_callback, 10
        )
        self.camera_image_sub = self.create_subscription(
            Image, '/swarm/camera/image', self.camera_image_callback, 10
        )
        
        # Test state
        self.status_received = False
        self.leader_pose_received = False
        self.robot_markers_received = False
        self.obstacles_received = False
        self.camera_image_received = False
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_tests)
        self.test_step = 0
        
        self.get_logger().info("🧪 Swarm Tester initialized")
    
    def status_callback(self, msg):
        """Callback for status messages"""
        self.status_received = True
        self.get_logger().info(f"📊 Status: {msg.data}")
    
    def leader_pose_callback(self, msg):
        """Callback for leader pose messages"""
        self.leader_pose_received = True
        self.get_logger().info(f"🎯 Leader pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
    
    def robot_markers_callback(self, msg):
        """Callback for robot markers"""
        self.robot_markers_received = True
        self.get_logger().info(f"🤖 Robot markers: {len(msg.markers)} robots")
    
    def obstacles_callback(self, msg):
        """Callback for obstacle markers"""
        self.obstacles_received = True
        self.get_logger().info(f"🚧 Obstacles: {len(msg.markers)} obstacles")
    
    def camera_image_callback(self, msg):
        """Callback for camera images"""
        self.camera_image_received = True
        self.get_logger().info(f"📷 Camera image: {msg.width}x{msg.height}")
    
    async def call_service(self, client, request):
        """Helper to call a service"""
        if not client.service_is_ready():
            self.get_logger().warn(f"Service {client.srv_name} not ready")
            return False
        
        future = client.call_async(request)
        try:
            response = await future
            self.get_logger().info(f"✅ Service {client.srv_name}: {response.data}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Service {client.srv_name} failed: {e}")
            return False
    
    async def run_tests(self):
        """Run the test sequence"""
        self.test_step += 1
        
        if self.test_step == 1:
            self.get_logger().info("🧪 Starting ROS2 Integration Tests...")
            
            # Test 1: Check if topics are publishing
            self.get_logger().info("📡 Testing topic subscriptions...")
            time.sleep(1)
            
            if self.status_received:
                self.get_logger().info("✅ Status topic working")
            else:
                self.get_logger().warn("⚠️ Status topic not received")
            
            if self.leader_pose_received:
                self.get_logger().info("✅ Leader pose topic working")
            else:
                self.get_logger().warn("⚠️ Leader pose topic not received")
            
            if self.robot_markers_received:
                self.get_logger().info("✅ Robot markers topic working")
            else:
                self.get_logger().warn("⚠️ Robot markers topic not received")
            
            if self.obstacles_received:
                self.get_logger().info("✅ Obstacles topic working")
            else:
                self.get_logger().warn("⚠️ Obstacles topic not received")
        
        elif self.test_step == 2:
            # Test 2: Test formation service
            self.get_logger().info("🔄 Testing formation service...")
            request = String()
            request.data = "line"
            await self.call_service(self.set_formation_client, request)
        
        elif self.test_step == 3:
            # Test 3: Test vision toggle
            self.get_logger().info("👁️ Testing vision toggle...")
            request = Bool()
            request.data = True
            await self.call_service(self.toggle_vision_client, request)
        
        elif self.test_step == 4:
            # Test 4: Test obstacles toggle
            self.get_logger().info("🚧 Testing obstacles toggle...")
            request = Bool()
            request.data = True
            await self.call_service(self.toggle_obstacles_client, request)
        
        elif self.test_step == 5:
            # Test 5: Test adding obstacle
            self.get_logger().info("➕ Testing add obstacle...")
            request = Float32MultiArray()
            request.data = [5.0, 5.0, 1.0, 0]  # x, y, radius, is_dynamic
            await self.call_service(self.add_obstacle_client, request)
        
        elif self.test_step == 6:
            # Test 6: Test controller change
            self.get_logger().info("🎛️ Testing controller change...")
            request = String()
            request.data = "MPC"
            await self.call_service(self.set_controller_client, request)
        
        elif self.test_step == 7:
            # Test 7: Check camera image
            self.get_logger().info("📷 Testing camera image...")
            time.sleep(1)
            
            if self.camera_image_received:
                self.get_logger().info("✅ Camera image topic working")
            else:
                self.get_logger().warn("⚠️ Camera image topic not received")
        
        elif self.test_step == 8:
            # Test 8: Final status check
            self.get_logger().info("📊 Final status check...")
            time.sleep(1)
            
            self.get_logger().info("🎉 ROS2 Integration Test Complete!")
            self.get_logger().info("📋 Summary:")
            self.get_logger().info(f"   Status topic: {'✅' if self.status_received else '❌'}")
            self.get_logger().info(f"   Leader pose: {'✅' if self.leader_pose_received else '❌'}")
            self.get_logger().info(f"   Robot markers: {'✅' if self.robot_markers_received else '❌'}")
            self.get_logger().info(f"   Obstacles: {'✅' if self.obstacles_received else '❌'}")
            self.get_logger().info(f"   Camera image: {'✅' if self.camera_image_received else '❌'}")
            
            # Stop the test
            self.test_timer.cancel()
            self.get_logger().info("🏁 Test completed. Press Ctrl+C to exit.")


def main(args=None):
    rclpy.init(args=args)
    
    tester = SwarmTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 