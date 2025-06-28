#!/usr/bin/env python3

"""
ROS2 Swarm Bridge Demo

This demo shows the progression from matplotlib-based simulation to ROS2:
- Converts the unified swarm system to use ROS2 topics
- Simulates what would happen with real Gazebo robots
- Demonstrates the bridge between Python algorithms and ROS2 ecosystem

Usage:
  source activate_swarm_ros2.sh
  python ros2_swarm_bridge.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class SwarmRobot:
    """Robot class with ROS2 integration."""
    
    def __init__(self, robot_id, x=0.0, y=0.0, theta=0.0, color='blue'):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.color = color
        self.cmd_v = 0.0
        self.cmd_omega = 0.0
        self.position_history = [(x, y)]
    
    def update(self, dt):
        """Update robot state."""
        self.v = self.cmd_v
        self.omega = self.cmd_omega
        
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.position_history.append((self.x, self.y))
        if len(self.position_history) > 100:
            self.position_history = self.position_history[-100:]
    
    def set_cmd_vel(self, linear_x, angular_z):
        """Set commanded velocities."""
        self.cmd_v = max(min(linear_x, 1.0), -1.0)
        self.cmd_omega = max(min(angular_z, 1.5), -1.5)

class SwarmControllerNode(Node):
    """ROS2 node for swarm formation control."""
    
    def __init__(self):
        super().__init__('swarm_controller')
        self.get_logger().info('🤖 ROS2 Swarm Controller Starting...')
        
        # Robot fleet
        self.robots = {
            'leader': SwarmRobot('leader', 0, 0, 0, 'red'),
            'follower_1': SwarmRobot('follower_1', -2, -1.5, 0, 'blue'),
            'follower_2': SwarmRobot('follower_2', -2, 1.5, 0, 'green'),
            'follower_3': SwarmRobot('follower_3', -4, 0, 0, 'orange')
        }
        
        # Formation parameters
        self.current_formation = 'triangle'
        self.formations = {
            'triangle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'line': [(-2.0, 0.0), (-4.0, 0.0), (-6.0, 0.0)],
            'circle': [(-2.1, -2.1), (-2.1, 2.1), (-4.2, 0.0)]
        }
        
        self.dt = 0.05
        self.time = 0.0
        self.leader_speed = 0.4
        self.leader_radius = 5.0
        
        # Create ROS2 publishers and subscribers
        self._create_ros2_interface()
        
        # Start simulation timer
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)
        
        # Start visualization
        self.viz_thread = threading.Thread(target=self._run_visualization, daemon=True)
        self.viz_thread.start()
        
        self.get_logger().info('✅ ROS2 Swarm Controller Ready!')
    
    def _create_ros2_interface(self):
        """Create ROS2 publishers and subscribers."""
        # Publishers for robot odometry
        self.odom_publishers = {}
        for robot_name in self.robots.keys():
            self.odom_publishers[robot_name] = self.create_publisher(
                Odometry, f'/{robot_name}/odom', 10
            )
        
        # Subscribers for robot commands
        self.cmd_vel_subscribers = {}
        for robot_name in self.robots.keys():
            self.cmd_vel_subscribers[robot_name] = self.create_subscription(
                Twist,
                f'/{robot_name}/cmd_vel',
                lambda msg, name=robot_name: self._cmd_vel_callback(msg, name),
                10
            )
        
        # Status publisher
        self.status_publisher = self.create_publisher(String, '/swarm/status', 10)
    
    def _cmd_vel_callback(self, msg, robot_name):
        """Handle incoming velocity commands."""
        if robot_name in self.robots:
            self.robots[robot_name].set_cmd_vel(msg.linear.x, msg.angular.z)
    
    def simulation_step(self):
        """Main simulation step."""
        self.time += self.dt
        
        # Update leader (autonomous circular motion)
        self._update_leader()
        
        # Update followers with formation control
        self._update_followers()
        
        # Update all robot physics
        for robot in self.robots.values():
            robot.update(self.dt)
        
        # Publish robot states
        self._publish_robot_states()
    
    def _update_leader(self):
        """Update leader robot."""
        leader = self.robots['leader']
        omega = self.leader_speed / self.leader_radius
        target_x = self.leader_radius * math.cos(omega * self.time)
        target_y = self.leader_radius * math.sin(omega * self.time)
        
        dx = target_x - leader.x
        dy = target_y - leader.y
        target_theta = math.atan2(dy, dx)
        
        angle_error = target_theta - leader.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        leader.set_cmd_vel(
            min(1.0 * math.sqrt(dx**2 + dy**2), 1.0),
            2.0 * angle_error
        )
    
    def _update_followers(self):
        """Update follower robots with formation control."""
        leader = self.robots['leader']
        leader_pos = np.array([leader.x, leader.y])
        
        formation_offsets = self.formations[self.current_formation]
        followers = ['follower_1', 'follower_2', 'follower_3']
        
        for i, follower_name in enumerate(followers):
            if i < len(formation_offsets):
                follower = self.robots[follower_name]
                offset = np.array(formation_offsets[i])
                target_pos = leader_pos + offset
                
                dx = target_pos[0] - follower.x
                dy = target_pos[1] - follower.y
                distance = math.sqrt(dx**2 + dy**2)
                target_angle = math.atan2(dy, dx)
                
                angle_error = target_angle - follower.theta
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                
                v_cmd = min(1.0 * distance, 1.0)
                omega_cmd = max(min(2.0 * angle_error, 1.5), -1.5)
                
                follower.set_cmd_vel(v_cmd, omega_cmd)
    
    def _publish_robot_states(self):
        """Publish robot odometry."""
        stamp = self.get_clock().now().to_msg()
        
        for robot_name, robot in self.robots.items():
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = f'{robot_name}/base_link'
            
            odom.pose.pose.position.x = robot.x
            odom.pose.pose.position.y = robot.y
            odom.pose.pose.position.z = 0.0
            
            odom.pose.pose.orientation.z = math.sin(robot.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(robot.theta / 2.0)
            
            odom.twist.twist.linear.x = robot.v
            odom.twist.twist.angular.z = robot.omega
            
            self.odom_publishers[robot_name].publish(odom)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Formation: {self.current_formation}, Time: {self.time:.1f}s'
        self.status_publisher.publish(status_msg)
    
    def _run_visualization(self):
        """Run matplotlib visualization."""
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 8))
        
        colors = {'leader': 'red', 'follower_1': 'blue', 'follower_2': 'green', 'follower_3': 'orange'}
        
        while rclpy.ok():
            try:
                ax.clear()
                ax.set_xlim(-8, 8)
                ax.set_ylim(-8, 8)
                ax.set_aspect('equal')
                ax.grid(True, alpha=0.3)
                ax.set_title(f'🤖 ROS2 Swarm Bridge Demo - Formation: {self.current_formation}', 
                           fontsize=14, fontweight='bold')
                
                for robot_name, robot in self.robots.items():
                    color = colors.get(robot_name, 'blue')
                    
                    # Robot body
                    circle = Circle((robot.x, robot.y), 0.25, color=color, alpha=0.7)
                    ax.add_patch(circle)
                    
                    # Robot orientation
                    dx = 0.3 * math.cos(robot.theta)
                    dy = 0.3 * math.sin(robot.theta)
                    ax.arrow(robot.x, robot.y, dx, dy, head_width=0.1, 
                           head_length=0.1, fc=color, ec=color)
                    
                    # Robot trail
                    if len(robot.position_history) > 1:
                        trail_x = [pos[0] for pos in robot.position_history[-30:]]
                        trail_y = [pos[1] for pos in robot.position_history[-30:]]
                        ax.plot(trail_x, trail_y, color=color, alpha=0.3, linewidth=1)
                    
                    ax.text(robot.x, robot.y - 0.5, robot_name, 
                           ha='center', fontsize=8, fontweight='bold')
                
                # Add ROS2 info
                info_text = (
                    f"📡 ROS2 Topics Active:\n"
                    f"• /leader/odom, /follower_*/odom\n"
                    f"• /leader/cmd_vel, /follower_*/cmd_vel\n"
                    f"• /swarm/status\n"
                    f"\n🎮 Try: ros2 topic list\n"
                    f"⏱️ Time: {self.time:.1f}s"
                )
                
                ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
                       verticalalignment='top', fontsize=9,
                       bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
                
                plt.pause(0.05)
                
            except Exception as e:
                break
        
        plt.ioff()

def main():
    print("🤖 Starting ROS2 Swarm Bridge Demo...")
    print("🔄 This bridges matplotlib simulation to ROS2 topics")
    print("📡 Robot states published on ROS2 topics")
    print()
    
    rclpy.init()
    
    try:
        node = SwarmControllerNode()
        print("🚀 Demo running! Try these commands:")
        print("   ros2 topic list")
        print("   ros2 topic echo /swarm/status")
        print("   ros2 topic echo /leader/odom")
        print()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 