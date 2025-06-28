#!/usr/bin/env python3

"""
ROS2 Swarm Bridge Demo (Headless Version)

This demo shows the progression from matplotlib-based simulation to ROS2
without matplotlib visualization to avoid macOS threading issues.

Features:
- Pure ROS2 node functionality
- Robot state publishing on ROS2 topics
- Formation control
- Can be visualized with rviz2 or ros2 topic echo

Usage:
  source activate_swarm_ros2.sh
  python ros2_swarm_bridge_headless.py
  
  # In another terminal:
  ros2 topic list
  ros2 topic echo /swarm/status
  ros2 topic echo /leader/odom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import time

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
        if len(self.position_history) > 50:
            self.position_history = self.position_history[-50:]
    
    def set_cmd_vel(self, linear_x, angular_z):
        """Set commanded velocities."""
        self.cmd_v = max(min(linear_x, 1.0), -1.0)
        self.cmd_omega = max(min(angular_z, 1.5), -1.5)

class SwarmControllerNode(Node):
    """ROS2 node for swarm formation control."""
    
    def __init__(self):
        super().__init__('swarm_controller')
        self.get_logger().info('🤖 ROS2 Swarm Controller Starting (Headless)...')
        
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
            'circle': [(-2.1, -2.1), (-2.1, 2.1), (-4.2, 0.0)],
            'v_shape': [(-2.5, -2.5), (-2.5, 2.5), (-5.0, 0.0)]
        }
        
        self.dt = 0.05
        self.time = 0.0
        self.leader_speed = 0.4
        self.leader_radius = 5.0
        
        # Statistics
        self.step_count = 0
        self.last_status_time = time.time()
        
        # Create ROS2 publishers and subscribers
        self._create_ros2_interface()
        
        # Start simulation timer
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)
        
        # Status reporting timer (every 5 seconds)
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('✅ ROS2 Swarm Controller Ready!')
        self.get_logger().info('📡 Publishing on topics: /leader/odom, /follower_*/odom, /swarm/status')
        self.get_logger().info('🎮 Try: ros2 topic list, ros2 topic echo /swarm/status')
    
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
        
        # Formation control subscriber
        self.formation_subscriber = self.create_subscription(
            String,
            '/swarm/set_formation',
            self._formation_callback,
            10
        )
    
    def _cmd_vel_callback(self, msg, robot_name):
        """Handle incoming velocity commands."""
        if robot_name in self.robots:
            self.robots[robot_name].set_cmd_vel(msg.linear.x, msg.angular.z)
            self.get_logger().debug(f'Received cmd_vel for {robot_name}: v={msg.linear.x:.2f}, ω={msg.angular.z:.2f}')
    
    def _formation_callback(self, msg):
        """Handle formation change requests."""
        formation = msg.data
        if formation in self.formations:
            self.current_formation = formation
            self.get_logger().info(f'🔄 Formation changed to: {formation}')
        else:
            self.get_logger().warn(f'❌ Unknown formation: {formation}')
    
    def simulation_step(self):
        """Main simulation step."""
        self.time += self.dt
        self.step_count += 1
        
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
        status_msg.data = f'Formation: {self.current_formation}, Time: {self.time:.1f}s, Steps: {self.step_count}'
        self.status_publisher.publish(status_msg)
    
    def print_status(self):
        """Print periodic status updates."""
        leader = self.robots['leader']
        self.get_logger().info(f'🔄 Status Update:')
        self.get_logger().info(f'   Formation: {self.current_formation}')
        self.get_logger().info(f'   Time: {self.time:.1f}s, Steps: {self.step_count}')
        self.get_logger().info(f'   Leader position: ({leader.x:.2f}, {leader.y:.2f})')
        
        # Calculate formation errors
        formation_offsets = self.formations[self.current_formation]
        followers = ['follower_1', 'follower_2', 'follower_3']
        
        for i, follower_name in enumerate(followers):
            if i < len(formation_offsets):
                follower = self.robots[follower_name]
                leader_pos = np.array([leader.x, leader.y])
                target_pos = leader_pos + np.array(formation_offsets[i])
                error = np.linalg.norm([follower.x - target_pos[0], follower.y - target_pos[1]])
                self.get_logger().info(f'   {follower_name} formation error: {error:.3f}m')

def main():
    print("🤖 Starting ROS2 Swarm Bridge Demo (Headless)...")
    print("🔄 This demonstrates matplotlib→ROS2 conversion without GUI")
    print("📡 Robot states will be published on ROS2 topics")
    print("🎮 No matplotlib GUI - use ROS2 tools for visualization")
    print()
    
    rclpy.init()
    
    try:
        node = SwarmControllerNode()
        print("🚀 Demo running! Try these commands in another terminal:")
        print("   ros2 topic list")
        print("   ros2 topic echo /swarm/status")
        print("   ros2 topic echo /leader/odom --once")
        print("   ros2 topic pub /swarm/set_formation std_msgs/String 'data: circle'")
        print("   ros2 topic pub /leader/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'")
        print()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Demo stopped cleanly")

if __name__ == '__main__':
    main() 