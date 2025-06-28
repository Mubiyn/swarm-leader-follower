#!/usr/bin/env python3
"""
Project Cleanup and Organization Script
Moves files to proper directories and removes duplicates
"""

import os
import shutil
from pathlib import Path

def organize_project():
    print("🧹 Organizing Modern Swarm Leader-Follower Project...")
    
    # Create organized directory structure
    dirs_to_create = [
        "demos/",           # All demo files
        "demos/python/",    # Pure Python demos  
        "demos/ros2/",      # ROS2 integrated demos
        "archive/",         # Old/experimental files
        "tests/unit/",      # Unit tests
        "tests/integration/", # Integration tests
    ]
    
    for dir_path in dirs_to_create:
        os.makedirs(dir_path, exist_ok=True)
        print(f"✅ Created: {dir_path}")
    
    # Files to move to demos/python/
    python_demos = [
        "simple_demo.py",
        "multi_follower_demo.py", 
        "vision_leader_follower.py",
        "obstacle_avoidance_demo.py",
        "mpc_leader_follower.py",
        "rl_leader_follower.py",
        "unified_swarm_system.py",
        "speed_test.py"
    ]
    
    # Files to move to demos/ros2/
    ros2_demos = [
        "clean_start.py",
        "multi_follower_ros2.py",
        "vision_leader_follower_ros2.py", 
        "obstacle_avoidance_ros2.py",
        "ros2_swarm_bridge.py",
        "ros2_swarm_bridge_fixed.py",
        "ros2_swarm_bridge_headless.py",
        "ros2_swarm_bridge_with_services.py"
    ]
    
    # Files to archive (experimental/duplicate versions)
    archive_files = [
        "test_individual_demos.py",
        "test_results.txt"
    ]
    
    # Move files
    for file in python_demos:
        if os.path.exists(file):
            shutil.move(file, f"demos/python/{file}")
            print(f"📦 Moved {file} to demos/python/")
    
    for file in ros2_demos:
        if os.path.exists(file):
            shutil.move(file, f"demos/ros2/{file}")
            print(f"🤖 Moved {file} to demos/ros2/")
    
    for file in archive_files:
        if os.path.exists(file):
            shutil.move(file, f"archive/{file}")
            print(f"📁 Archived {file}")
    
    # Keep core files in root
    core_files = [
        "test_all_demos.py",  # Main entry point
        "README.md",
        "README_CLEAN.md", 
        "GETTING_STARTED.md",
        "GETTING_STARTED_MACOS.md",
        "PROJECT_FINAL_SUMMARY.md",
        "requirements.txt",
        "requirements-conda.txt",
        "activate_swarm_ros2.sh"
    ]
    
    print("\n🎯 Core files remaining in root:")
    for file in core_files:
        if os.path.exists(file):
            print(f"  ✅ {file}")
    
    print("\n🏗️ Directory structure now:")
    print("modern_swarm_leader_follower/")
    print("├── demos/")
    print("│   ├── python/     # Pure Python demos")
    print("│   └── ros2/       # ROS2 integrated demos")
    print("├── src/            # ROS2 package source")
    print("├── launch/         # ROS2 launch files")
    print("├── config/         # Configuration files")
    print("├── urdf/           # Robot descriptions")
    print("├── worlds/         # Gazebo worlds")
    print("├── archive/        # Old experimental files")
    print("└── tests/          # Test files")
    
    print("\n✅ Project organization complete!")

if __name__ == "__main__":
    organize_project() 