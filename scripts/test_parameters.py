#!/usr/bin/env python3
"""
🧪 Parameter Testing Script for Enhanced ROS2 Swarm System

This script demonstrates the comprehensive parameter system capabilities.
Use this to test and validate parameter changes in real-time.

Usage:
    python scripts/test_parameters.py
    
Requirements:
    - Enhanced ROS2 swarm system running
    - ROS2 environment activated
"""

import rclpy
from rclpy.node import Node
import time
import subprocess
import sys

class ParameterTester(Node):
    """🧪 Parameter testing and demonstration node."""
    
    def __init__(self):
        super().__init__('parameter_tester')
        self.get_logger().info("🧪 Parameter Testing System Starting...")
        
    def run_parameter_test(self, param_name, value, description=""):
        """Test a single parameter change."""
        try:
            cmd = f"ros2 param set /swarm_controller {param_name} {value}"
            result = subprocess.run(cmd.split(), capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info(f"✅ {description}: {param_name} = {value}")
                return True
            else:
                self.get_logger().error(f"❌ Failed: {param_name} = {value}")
                self.get_logger().error(f"   Error: {result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception testing {param_name}: {e}")
            return False
    
    def get_parameter_value(self, param_name):
        """Get current parameter value."""
        try:
            cmd = f"ros2 param get /swarm_controller {param_name}"
            result = subprocess.run(cmd.split(), capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout.strip()
            return "UNKNOWN"
        except:
            return "ERROR"
    
    def run_comprehensive_test(self):
        """Run comprehensive parameter testing."""
        
        self.get_logger().info("🚀 Starting Comprehensive Parameter Testing!")
        self.get_logger().info("=" * 60)
        
        # Test formation parameters
        self.get_logger().info("📐 Testing Formation Parameters...")
        formation_tests = [
            ("formation.spacing", "1.5", "Tight formation spacing"),
            ("formation.spacing", "3.0", "Wide formation spacing"),
            ("formation.max_followers", "4", "Increase max followers"),
            ("formation.switch_threshold", "1.0", "Increase switch threshold"),
            ("formation.adaptive_spacing", "false", "Disable adaptive spacing"),
        ]
        
        for param, value, desc in formation_tests:
            self.run_parameter_test(param, value, desc)
            time.sleep(1)
        
        time.sleep(2)
        
        # Test control parameters
        self.get_logger().info("🎛️ Testing Control Parameters...")
        control_tests = [
            ("control.proportional_gain", "1.5", "Increase P-gain"),
            ("control.max_velocity", "0.5", "Reduce max velocity"),
            ("control.max_angular_velocity", "2.0", "Increase max angular velocity"),
            ("control.velocity_smoothing", "0.2", "Increase smoothing"),
        ]
        
        for param, value, desc in control_tests:
            self.run_parameter_test(param, value, desc)
            time.sleep(1)
        
        time.sleep(2)
        
        # Test leader parameters
        self.get_logger().info("🔄 Testing Leader Parameters...")
        leader_tests = [
            ("leader.radius", "3.0", "Smaller leader radius"),
            ("leader.speed", "1.5", "Faster leader speed"),
            ("leader.circular_motion", "false", "Disable circular motion"),
            ("leader.motion_pattern", "figure8", "Change to figure-8 pattern"),
        ]
        
        for param, value, desc in leader_tests:
            self.run_parameter_test(param, value, desc)
            time.sleep(1)
        
        time.sleep(2)
        
        # Test obstacle parameters
        self.get_logger().info("🚧 Testing Obstacle Parameters...")
        obstacle_tests = [
            ("obstacles.avoidance_distance", "2.0", "Increase avoidance distance"),
            ("obstacles.repulsion_gain", "3.0", "Stronger repulsion"),
            ("obstacles.max_obstacles", "15", "More obstacles allowed"),
            ("obstacles.dynamic_probability", "0.7", "More dynamic obstacles"),
        ]
        
        for param, value, desc in obstacle_tests:
            self.run_parameter_test(param, value, desc)
            time.sleep(1)
        
        time.sleep(2)
        
        # Test visualization parameters
        self.get_logger().info("📺 Testing Visualization Parameters...")
        viz_tests = [
            ("visualization.trail_length", "50", "Shorter trails"),
            ("visualization.update_rate", "30.0", "Faster updates"),
            ("visualization.show_trajectories", "false", "Hide trajectories"),
            ("visualization.robot_size", "0.3", "Larger robots"),
        ]
        
        for param, value, desc in viz_tests:
            self.run_parameter_test(param, value, desc)
            time.sleep(1)
        
        time.sleep(2)
        
        # Test safety parameters
        self.get_logger().info("🛡️ Testing Safety Parameters...")
        safety_tests = [
            ("safety.collision_distance", "0.5", "Larger collision distance"),
            ("safety.boundary_x", "6.0", "Smaller X boundary"),
            ("safety.boundary_y", "6.0", "Smaller Y boundary"),
            ("safety.boundary_enforcement", "false", "Disable boundaries"),
        ]
        
        for param, value, desc in safety_tests:
            self.run_parameter_test(param, value, desc)
            time.sleep(1)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Comprehensive Parameter Testing Complete!")
        
    def run_scenario_tests(self):
        """Test specific scenarios with parameter combinations."""
        
        self.get_logger().info("🎬 Testing Parameter Scenarios...")
        self.get_logger().info("=" * 60)
        
        scenarios = [
            {
                "name": "🏃 High-Speed Mode",
                "params": [
                    ("leader.speed", "2.0"),
                    ("control.max_velocity", "2.0"),
                    ("control.proportional_gain", "1.5"),
                ]
            },
            {
                "name": "🎯 Precision Mode", 
                "params": [
                    ("formation.spacing", "1.0"),
                    ("control.velocity_smoothing", "0.05"),
                    ("safety.collision_distance", "0.2"),
                ]
            },
            {
                "name": "🚧 Obstacle-Heavy Environment",
                "params": [
                    ("obstacles.avoidance_distance", "2.5"),
                    ("obstacles.repulsion_gain", "4.0"),
                    ("obstacles.max_obstacles", "20"),
                ]
            },
            {
                "name": "📊 Performance Mode",
                "params": [
                    ("performance.simulation_rate", "50.0"),
                    ("visualization.update_rate", "25.0"),
                    ("visualization.trail_length", "30"),
                ]
            }
        ]
        
        for scenario in scenarios:
            self.get_logger().info(f"Testing scenario: {scenario['name']}")
            
            for param, value in scenario['params']:
                self.run_parameter_test(param, value, f"  {scenario['name']}")
                time.sleep(0.5)
            
            self.get_logger().info(f"✅ Scenario {scenario['name']} configured!")
            time.sleep(3)  # Let the scenario run for a bit
            
        self.get_logger().info("✅ All scenario testing complete!")
    
    def display_current_parameters(self):
        """Display current parameter values."""
        
        self.get_logger().info("📊 Current Parameter Values:")
        self.get_logger().info("=" * 60)
        
        param_groups = {
            "Formation": [
                "formation.spacing",
                "formation.max_followers", 
                "formation.switch_threshold",
                "formation.adaptive_spacing"
            ],
            "Control": [
                "control.proportional_gain",
                "control.max_velocity",
                "control.max_angular_velocity",
                "control.velocity_smoothing"
            ],
            "Leader": [
                "leader.radius",
                "leader.speed", 
                "leader.circular_motion",
                "leader.motion_pattern"
            ],
            "Safety": [
                "safety.collision_distance",
                "safety.boundary_x",
                "safety.boundary_y",
                "safety.emergency_stop"
            ]
        }
        
        for group, params in param_groups.items():
            self.get_logger().info(f"📋 {group} Parameters:")
            for param in params:
                value = self.get_parameter_value(param)
                self.get_logger().info(f"   • {param}: {value}")
            self.get_logger().info("")

def main():
    """Main testing function."""
    
    print("🧪 Enhanced ROS2 Swarm Parameter Testing System")
    print("=" * 60)
    print("This script will test the comprehensive parameter system.")
    print("Make sure the enhanced swarm system is running!")
    print("=" * 60)
    
    # Check if system is running
    try:
        result = subprocess.run(
            ["ros2", "node", "list"], 
            capture_output=True, text=True, timeout=5
        )
        if "/swarm_controller" not in result.stdout:
            print("❌ ERROR: Swarm controller not running!")
            print("   Start with: python ros2_swarm_bridge_with_services.py")
            return
    except:
        print("❌ ERROR: ROS2 not available or timeout!")
        return
    
    rclpy.init()
    tester = ParameterTester()
    
    try:
        # Display current parameters
        tester.display_current_parameters()
        time.sleep(2)
        
        # Ask user what to test
        print("\nWhat would you like to test?")
        print("1. 🧪 Comprehensive parameter testing")
        print("2. 🎬 Scenario-based testing") 
        print("3. 📊 Display current parameters only")
        print("4. 🔧 Interactive parameter setting")
        
        choice = input("Enter choice (1-4): ").strip()
        
        if choice == "1":
            tester.run_comprehensive_test()
        elif choice == "2":
            tester.run_scenario_tests()
        elif choice == "3":
            tester.display_current_parameters()
        elif choice == "4":
            print("🔧 Interactive Parameter Setting")
            while True:
                param = input("Enter parameter name (or 'quit'): ").strip()
                if param == 'quit':
                    break
                value = input(f"Enter value for {param}: ").strip()
                tester.run_parameter_test(param, value, "Interactive")
        else:
            print("Invalid choice!")
            
    except KeyboardInterrupt:
        print("\n🛑 Testing interrupted by user")
    except Exception as e:
        print(f"❌ Error during testing: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print("🧪 Parameter testing complete!")

if __name__ == '__main__':
    main() 