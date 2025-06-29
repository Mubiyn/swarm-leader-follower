#!/usr/bin/env python3
"""
Test Enhanced Performance Monitoring

This script demonstrates the enhanced performance monitoring features:
- Formation error tracking over time
- Collision statistics
- Real-time performance analysis
- System health assessment
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
import json

class PerformanceTester(Node):
    def __init__(self):
        super().__init__('performance_tester')
        
        # Create service clients
        self.toggle_vision_client = self.create_client(SetBool, '/swarm/toggle_vision')
        self.toggle_vision_detection_client = self.create_client(SetBool, '/swarm/toggle_vision_detection')
        self.set_controller_client = self.create_client(SetBool, '/swarm/set_controller')
        self.set_formation_client = self.create_client(SetBool, '/swarm/set_formation')
        self.generate_report_client = self.create_client(SetBool, '/swarm/logging/generate_report')
        
        self.get_logger().info("🎯 Performance Tester initialized!")
        self.get_logger().info("📊 Testing enhanced performance monitoring features...")
    
    def test_vision_and_mpc(self):
        """Test vision and MPC services with performance monitoring"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🎥 Testing Vision System...")
        
        # Enable vision system
        self.call_service(self.toggle_vision_client, True, "Enable synthetic vision")
        time.sleep(1)
        
        # Enable vision detection
        self.call_service(self.toggle_vision_detection_client, True, "Enable vision detection")
        time.sleep(1)
        
        self.get_logger().info("🧠 Testing MPC Controller...")
        
        # Switch to MPC controller
        self.call_service(self.set_controller_client, True, "Switch to MPC controller")
        time.sleep(2)
        
        # Test different formations
        self.get_logger().info("📐 Testing Different Formations...")
        formations = ["triangle", "line", "circle", "v-shape"]
        
        for formation in formations:
            self.get_logger().info(f"Testing {formation} formation...")
            self.call_service(self.set_formation_client, True, f"Switch to {formation} formation")
            time.sleep(3)  # Let formation stabilize
        
        # Generate performance report
        self.get_logger().info("📊 Generating Performance Report...")
        self.call_service(self.generate_report_client, True, "Generate comprehensive report")
        time.sleep(2)
        
        self.get_logger().info("✅ Vision and MPC testing completed!")
    
    def call_service(self, client, data, description):
        """Helper function to call services"""
        try:
            request = SetBool.Request()
            request.data = data
            
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                self.get_logger().info(f"✅ {description}: Success")
            else:
                self.get_logger().warn(f"⚠️ {description}: Failed")
                
        except Exception as e:
            self.get_logger().error(f"❌ {description}: Error - {e}")
    
    def monitor_performance(self, duration=30):
        """Monitor performance metrics for a specified duration"""
        self.get_logger().info(f"\n📈 Monitoring Performance for {duration} seconds...")
        self.get_logger().info("Watch for formation errors and collision statistics in the logs!")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(5)
            elapsed = time.time() - start_time
            self.get_logger().info(f"⏱️ Monitoring: {elapsed:.1f}s / {duration}s")
        
        self.get_logger().info("✅ Performance monitoring completed!")

def main():
    rclpy.init()
    
    tester = PerformanceTester()
    
    try:
        # Test vision and MPC services
        tester.test_vision_and_mpc()
        
        # Monitor performance
        tester.monitor_performance(30)
        
        tester.get_logger().info("\n" + "="*60)
        tester.get_logger().info("🎉 Enhanced Performance Monitoring Test Complete!")
        tester.get_logger().info("\n📊 What to check:")
        tester.get_logger().info("   • Formation errors over time in logs")
        tester.get_logger().info("   • Collision statistics and rates")
        tester.get_logger().info("   • System health assessment")
        tester.get_logger().info("   • Performance recommendations")
        tester.get_logger().info("   • Vision system metrics")
        tester.get_logger().info("   • MPC controller performance")
        
    except KeyboardInterrupt:
        tester.get_logger().info("🛑 Test interrupted by user")
    except Exception as e:
        tester.get_logger().error(f"❌ Test error: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 