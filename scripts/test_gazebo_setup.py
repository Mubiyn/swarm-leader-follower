#!/usr/bin/env python3

"""
Test Script for Gazebo Swarm Setup

This script tests the Gazebo simulation setup to ensure:
- Robot models are valid
- World files are accessible
- Launch files are properly configured
- ROS2 environment is correctly set up

Usage:
  python scripts/test_gazebo_setup.py
"""

import os
import sys
import subprocess
import time

def test_file_exists(filepath, description):
    """Test if a file exists and report result."""
    if os.path.exists(filepath):
        print(f"✅ {description}: {filepath}")
        return True
    else:
        print(f"❌ {description}: {filepath} (NOT FOUND)")
        return False

def test_ros2_environment():
    """Test if ROS2 environment is properly set up."""
    print("\n🔍 Testing ROS2 Environment...")
    
    try:
        # Test ROS2 command
        result = subprocess.run(['ros2', '--help'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ ROS2 command available")
        else:
            print("❌ ROS2 command failed")
            return False
    except Exception as e:
        print(f"❌ ROS2 not available: {e}")
        return False
    
    try:
        # Test Gazebo command
        result = subprocess.run(['gazebo', '--version'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ Gazebo available: {result.stdout.strip()}")
        else:
            print("❌ Gazebo command failed")
            return False
    except Exception as e:
        print(f"❌ Gazebo not available: {e}")
        return False
    
    return True

def test_urdf_validation():
    """Test URDF file validation."""
    print("\n🔍 Testing URDF Validation...")
    
    urdf_path = "urdf/swarm_robot.urdf.xacro"
    if not test_file_exists(urdf_path, "Robot URDF file"):
        return False
    
    try:
        # Test xacro processing
        result = subprocess.run([
            'xacro', urdf_path, 
            'robot_name:=test_robot',
            'robot_color:=blue'
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print("✅ URDF xacro processing successful")
            
            # Basic validation of output
            urdf_content = result.stdout
            if '<robot' in urdf_content and '</robot>' in urdf_content:
                print("✅ Generated URDF structure valid")
                return True
            else:
                print("❌ Generated URDF structure invalid")
                return False
        else:
            print(f"❌ URDF xacro processing failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ URDF validation error: {e}")
        return False

def test_world_files():
    """Test world file accessibility."""
    print("\n🔍 Testing World Files...")
    
    world_path = "worlds/swarm_arena.world"
    if not test_file_exists(world_path, "Swarm arena world file"):
        return False
    
    try:
        # Basic XML validation
        with open(world_path, 'r') as f:
            content = f.read()
            
        if '<world' in content and '</world>' in content:
            print("✅ World file XML structure valid")
        else:
            print("❌ World file XML structure invalid")
            return False
            
        if 'swarm_arena' in content:
            print("✅ World file contains swarm arena")
        else:
            print("❌ World file missing swarm arena")
            return False
            
        return True
        
    except Exception as e:
        print(f"❌ World file validation error: {e}")
        return False

def test_launch_files():
    """Test launch file syntax."""
    print("\n🔍 Testing Launch Files...")
    
    launch_files = [
        ("launch/gazebo_swarm_simulation.launch.py", "Gazebo simulation launch"),
        ("launch/formation_control.launch.py", "Formation control launch")
    ]
    
    all_valid = True
    
    for filepath, description in launch_files:
        if test_file_exists(filepath, description):
            try:
                # Basic Python syntax check
                with open(filepath, 'r') as f:
                    content = f.read()
                    
                compile(content, filepath, 'exec')
                print(f"✅ {description}: Python syntax valid")
                
                # Check for required functions
                if 'generate_launch_description' in content:
                    print(f"✅ {description}: Contains launch description function")
                else:
                    print(f"❌ {description}: Missing launch description function")
                    all_valid = False
                    
            except SyntaxError as e:
                print(f"❌ {description}: Python syntax error: {e}")
                all_valid = False
            except Exception as e:
                print(f"❌ {description}: Validation error: {e}")
                all_valid = False
        else:
            all_valid = False
    
    return all_valid

def create_test_summary():
    """Create a summary of test results."""
    print("\n" + "="*60)
    print("🧪 GAZEBO SWARM SETUP TEST RESULTS")
    print("="*60)
    
    tests = [
        (test_ros2_environment, "ROS2 Environment"),
        (test_urdf_validation, "Robot URDF Models"),
        (test_world_files, "Gazebo World Files"),
        (test_launch_files, "Launch Files")
    ]
    
    results = []
    for test_func, test_name in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name}: Test failed with exception: {e}")
            results.append((test_name, False))
    
    print("\n📊 SUMMARY:")
    print("-" * 30)
    
    passed = 0
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        emoji = "✅" if result else "❌"
        print(f"{emoji} {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n📈 Overall: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("\n🎉 All tests passed! Your Gazebo swarm setup is ready.")
        print("\n🚀 Next steps:")
        print("   1. ros2 launch launch/gazebo_swarm_simulation.launch.py")
        print("   2. ros2 launch launch/formation_control.launch.py")
        return True
    else:
        print(f"\n⚠️  {len(results) - passed} test(s) failed. Please fix the issues above.")
        return False

def main():
    """Main test function."""
    print("🧪 Gazebo Swarm Setup Test")
    print("=" * 40)
    print("Testing your ROS2 + Gazebo robot models and launch files...\n")
    
    # Change to project directory
    if os.path.basename(os.getcwd()) != 'modern_swarm_leader_follower':
        project_dirs = [
            '/Users/Mubiyn/Desktop/rp_2/modern_swarm_leader_follower',
            '../modern_swarm_leader_follower',
            './modern_swarm_leader_follower'
        ]
        
        for proj_dir in project_dirs:
            if os.path.exists(proj_dir):
                os.chdir(proj_dir)
                print(f"📁 Changed to project directory: {os.getcwd()}")
                break
        else:
            print("❌ Could not find project directory")
            return False
    
    # Run all tests and create summary
    success = create_test_summary()
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 