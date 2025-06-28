#!/usr/bin/env python3
"""
Speed Test - Quick verification of robot speeds
"""

import time
from simple_demo import SimpleLeaderFollower

def test_speeds():
    """Test and display robot speeds"""
    print("🔍 Testing Robot Speeds...")
    print("=" * 40)
    
    sim = SimpleLeaderFollower()
    
    # Test for 5 seconds
    start_time = time.time()
    max_linear_vel = 0
    max_angular_vel = 0
    
    while time.time() - start_time < 5:
        sim.update_leader()
        linear_vel, angular_vel, distance = sim.update_follower()
        
        max_linear_vel = max(max_linear_vel, abs(linear_vel))
        max_angular_vel = max(max_angular_vel, abs(angular_vel))
        
        time.sleep(0.1)
    
    print(f"📊 Speed Test Results:")
    print(f"   Max Linear Velocity:  {max_linear_vel:.2f} m/s")
    print(f"   Max Angular Velocity: {max_angular_vel:.2f} rad/s")
    print(f"   Leader Speed:         ~0.3 rad/s (circular)")
    print()
    
    # Check if speeds are reasonable
    if max_linear_vel <= 1.0 and max_angular_vel <= 1.5:
        print("✅ Robot speeds are now REASONABLE!")
        print("   - Linear velocity ≤ 1.0 m/s (walking pace)")
        print("   - Angular velocity ≤ 1.5 rad/s (smooth turning)")
    else:
        print("⚠️  Speeds might still be too high")
        
    print()
    print("🎯 Speed Improvements Made:")
    print("   - Leader circular motion: 0.5 → 0.3 rad/s")
    print("   - Control gains: k_linear 1.0 → 0.3, k_angular 2.0 → 1.0")
    print("   - Velocity limits: ±2.0 → ±1.0 m/s linear, ±2.0 → ±1.5 rad/s angular")
    print("   - Collision avoidance strength: 2.0 → 1.0")

if __name__ == '__main__':
    test_speeds() 