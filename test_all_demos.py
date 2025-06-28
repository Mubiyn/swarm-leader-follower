#!/usr/bin/env python3
"""
TEST ALL DEMOS - Interactive demo selector
Allows you to test all working implementations.
"""

import subprocess
import sys
import time

def print_banner():
    """Print banner"""
    print("=" * 60)
    print("🤖 MULTI-ROBOT LEADER-FOLLOWER SYSTEM DEMOS 🤖")
    print("=" * 60)
    print()

def print_menu():
    """Print demo menu"""
    print("Available Demos:")
    print()
    print("1. 🔵 Simple Demo (1 Leader + 1 Follower)")
    print("   - Pure Python, no ROS2 required")
    print("   - Basic leader-follower behavior")
    print("   - File: simple_demo.py")
    print()
    print("2. 🔴 Clean Start ROS2 (1 Leader + 1 Follower)")
    print("   - Full ROS2 implementation")
    print("   - Proper node communication")
    print("   - File: clean_start.py")
    print()
    print("3. 🔵🟢🟠 Multi-Follower + Formation Switching (1 Leader + 3 Followers)")
    print("   - Pure Python, no ROS2 required")
    print("   - 4 formation patterns: Triangle, Line, Circle, V-Shape")
    print("   - Press SPACEBAR to switch formations live!")
    print("   - File: multi_follower_demo.py")
    print()
    print("4. 🚀 Multi-Follower ROS2 (1 Leader + 3 Followers)")
    print("   - Full ROS2 implementation")
    print("   - Formation control with collision avoidance")
    print("   - File: multi_follower_ros2.py")
    print()
    print("5. 🎥 Vision-Based Leader Detection (Computer Vision)")
    print("   - Camera-based leader detection")
    print("   - Press V to toggle vision ON/OFF")
    print("   - Yellow X shows detected leader position")
    print("   - File: vision_leader_follower.py")
    print()
    print("6. 📊 Show Project Status")
    print("7. 🚧 Obstacle Avoidance (Static + Dynamic obstacles)")
    print("   - Navigate around obstacles while keeping formation")
    print("   - Static red circles and moving purple circles")
    print("   - Collision detection and avoidance forces")
    print("   - File: obstacle_avoidance_demo.py")
    print()
    print("8. 🎥 Vision ROS2 System (Computer Vision + ROS2)")
    print("   - Complete ROS2 vision-based multi-robot system")
    print("   - Separate nodes for vision, control, and robots")
    print("   - File: vision_leader_follower_ros2.py")
    print()
    print("9. 🚧 Obstacle Avoidance ROS2 (Full ROS2 Implementation)")
    print("   - Complete ROS2 obstacle avoidance system")
    print("   - Proper node separation and topic communication")
    print("   - File: obstacle_avoidance_ros2.py")
    print()
    print("10. 🧠 MPC vs Proportional Control (Advanced Control)")
    print("   - Model Predictive Control vs traditional control")
    print("   - Real-time performance comparison")
    print("   - Formation switching with optimized control")
    print("   - File: mpc_leader_follower.py")
    print()
    print("11. 🤖 RL vs Proportional Control (Reinforcement Learning)")
    print("   - Deep Q-Network based formation control")
    print("   - Real-time learning and adaptation")
    print("   - Training mode with model save/load")
    print("   - File: rl_leader_follower.py")
    print()
    print("12. ⚡ Test Robot Speeds")
    print("13. 🛑 Exit")
    print()

def run_demo(demo_file):
    """Run a demo file"""
    print(f"🚀 Starting {demo_file}...")
    print("Press Ctrl+C to stop the demo and return to menu")
    print("-" * 50)
    
    try:
        # Activate environment and run demo
        cmd = f"source activate_swarm_ros2.sh && python {demo_file}"
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError:
        print(f"❌ Error running {demo_file}")
    except KeyboardInterrupt:
        print(f"\n🛑 {demo_file} stopped by user")
    
    print("\nReturning to menu...")
    time.sleep(1)

def show_status():
    """Show current project status"""
    print("\n📊 PROJECT STATUS:")
    print("-" * 40)
    print("✅ Phase 1: Multiple Followers - COMPLETED")
    print("   - 1 leader + 3 followers working")
    print("   - Formation control implemented")
    print("   - Collision avoidance working")
    print("   - Both Python and ROS2 versions")
    print()
    print("✅ Phase 2: Formation Patterns - COMPLETED")
    print("   - 4 formation patterns: Triangle, Line, Circle, V-Shape")
    print("   - Live formation switching with SPACEBAR")
    print("   - Smooth formation transitions")
    print()
    print("✅ Phase 3: Computer Vision - COMPLETED")
    print("   - Camera-based leader detection implemented")
    print("   - Color-based detection working")
    print("   - Vision toggle functionality (V key)")
    print("   - Synthetic camera view with real-time detection")
    print()
    print("✅ Phase 4: Obstacle Avoidance - COMPLETED")
    print("   - Static obstacle avoidance implemented")
    print("   - Dynamic moving obstacle avoidance working")
    print("   - Formation maintenance during avoidance")
    print("   - Collision detection and safety metrics")
    print("   - Complete ROS2 implementation")
    print()
    print("✅ Phase 5: Advanced Control - COMPLETED")
    print("   - Model Predictive Control (MPC) implemented")
    print("   - Reinforcement Learning (DQN) working")
    print("   - Real-time comparison with traditional control")
    print("   - Performance metrics and optimization")
    print()
    print("📋 Upcoming Phases:")
    print("   - Real Robot Deployment")
    print("   - Advanced path planning algorithms")
    print()

def main():
    """Main menu loop"""
    while True:
        print_banner()
        print_menu()
        
        try:
            choice = input("Select demo (1-13): ").strip()
            
            if choice == '1':
                run_demo('simple_demo.py')
            elif choice == '2':
                run_demo('clean_start.py')
            elif choice == '3':
                run_demo('multi_follower_demo.py')
            elif choice == '4':
                run_demo('multi_follower_ros2.py')
            elif choice == '5':
                run_demo('vision_leader_follower.py')
            elif choice == '6':
                show_status()
                input("\nPress Enter to continue...")
            elif choice == '7':
                run_demo('obstacle_avoidance_demo.py')
            elif choice == '8':
                run_demo('vision_leader_follower_ros2.py')
            elif choice == '9':
                run_demo('obstacle_avoidance_ros2.py')
            elif choice == '10':
                run_demo('mpc_leader_follower.py')
            elif choice == '11':
                run_demo('rl_leader_follower.py')
            elif choice == '12':
                run_demo('speed_test.py')
                input("\nPress Enter to continue...")
            elif choice == '13':
                print("👋 Goodbye!")
                break
            else:
                print("❌ Invalid choice. Please select 1-13.")
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            time.sleep(1)

if __name__ == '__main__':
    main() 