#!/usr/bin/env python3
"""
🤖 ROS2 Swarm Bridge Demo (Threading Fixed)

A working ROS2 swarm system that bridges matplotlib simulation with ROS2 topics.
This version fixes the matplotlib threading issue on macOS.

Fixed: NSWindow threading error by running matplotlib in main thread.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Qt5Agg')  # Use Qt backend explicitly
from matplotlib.patches import Circle
import math
import threading
import queue
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class SwarmRobot:
    """Individual robot in the swarm."""
    
    def __init__(self, robot_id, x=0.0, y=0.0, theta=0.0, color='blue'):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0  # linear velocity
        self.omega = 0.0  # angular velocity
        self.color = color
        self.position_history = [(x, y)]
        
        # Control inputs
        self.cmd_v = 0.0
        self.cmd_omega = 0.0
    
    def update(self, dt):
        """Update robot physics."""
        self.v = self.cmd_v
        self.omega = self.cmd_omega
        
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Store position history
        self.position_history.append((self.x, self.y))
        if len(self.position_history) > 100:
            self.position_history.pop(0)
    
    def set_cmd_vel(self, linear_x, angular_z):
        """Set velocity commands."""
        self.cmd_v = max(min(linear_x, 1.0), -1.0)
        self.cmd_omega = max(min(angular_z, 1.5), -1.5)

class SwarmControllerNode(Node):
    """ROS2 node that controls the swarm and publishes to topics."""
    
    def __init__(self, visualization_queue):
        super().__init__('swarm_controller')
        
        # Simulation parameters
        self.dt = 0.05  # 20 Hz simulation
        self.time = 0.0
        self.visualization_queue = visualization_queue
        
        # Leader motion parameters
        self.leader_radius = 5.0
        self.leader_speed = 1.0
        
        # Formation configurations
        self.formations = {
            'triangle': [[-2.0, -1.5], [-2.0, 1.5], [-3.5, 0.0]],
            'line': [[-2.0, 0.0], [-4.0, 0.0], [-6.0, 0.0]],
            'circle': [
                [-2.0 * math.cos(i * 2*math.pi/3), -2.0 * math.sin(i * 2*math.pi/3)] 
                for i in range(3)
            ]
        }
        self.current_formation = 'triangle'
        
        # Create robots
        self.robots = {
            'leader': SwarmRobot('leader', 5.0, 0.0, 0.0, 'red'),
            'follower_1': SwarmRobot('follower_1', 3.0, -1.5, 0.0, 'blue'),
            'follower_2': SwarmRobot('follower_2', 3.0, 1.5, 0.0, 'green'),
            'follower_3': SwarmRobot('follower_3', 1.5, 0.0, 0.0, 'orange')
        }
        
        # Create ROS2 interface
        self._create_ros2_interface()
        
        # Start simulation timer
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)
        
        self.get_logger().info("🤖 ROS2 Swarm Controller Starting...")
        self.get_logger().info("✅ ROS2 Swarm Controller Ready!")
    
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
        
        # Send visualization data to main thread
        self._send_visualization_data()
    
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
    
    def _send_visualization_data(self):
        """Send robot data to visualization thread."""
        viz_data = {
            'time': self.time,
            'formation': self.current_formation,
            'robots': {}
        }
        
        for robot_name, robot in self.robots.items():
            viz_data['robots'][robot_name] = {
                'x': robot.x,
                'y': robot.y,
                'theta': robot.theta,
                'color': robot.color,
                'position_history': robot.position_history.copy()
            }
        
        # Send to visualization queue (non-blocking)
        try:
            self.visualization_queue.put_nowait(viz_data)
        except queue.Full:
            pass  # Skip if queue is full

class SwarmVisualization:
    """Main thread visualization using matplotlib."""
    
    def __init__(self, visualization_queue):
        self.visualization_queue = visualization_queue
        self.running = True
        
        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        
        # Default data
        self.current_data = None
        
    def run(self):
        """Main visualization loop."""
        print("🎮 Starting matplotlib visualization...")
        
        while self.running:
            try:
                # Get latest data from queue
                try:
                    while True:
                        self.current_data = self.visualization_queue.get_nowait()
                except queue.Empty:
                    pass
                
                # Update plot if we have data
                if self.current_data:
                    self._update_plot()
                
                # Small delay to prevent excessive CPU usage
                plt.pause(0.05)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Visualization error: {e}")
                break
        
        plt.ioff()
        plt.close('all')
    
    def _update_plot(self):
        """Update the matplotlib plot."""
        data = self.current_data
        
        self.ax.clear()
        self.ax.set_xlim(-8, 8)
        self.ax.set_ylim(-8, 8)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(f'🤖 ROS2 Swarm Bridge Demo - Formation: {data["formation"]}', 
                         fontsize=14, fontweight='bold')
        
        # Draw robots
        for robot_name, robot_data in data['robots'].items():
            color = robot_data['color']
            x, y, theta = robot_data['x'], robot_data['y'], robot_data['theta']
            
            # Robot body
            circle = Circle((x, y), 0.25, color=color, alpha=0.7)
            self.ax.add_patch(circle)
            
            # Robot orientation arrow
            dx = 0.3 * math.cos(theta)
            dy = 0.3 * math.sin(theta)
            self.ax.arrow(x, y, dx, dy, head_width=0.1, 
                         head_length=0.1, fc=color, ec=color)
            
            # Robot trail
            history = robot_data['position_history']
            if len(history) > 1:
                trail_x = [pos[0] for pos in history[-30:]]
                trail_y = [pos[1] for pos in history[-30:]]
                self.ax.plot(trail_x, trail_y, color=color, alpha=0.3, linewidth=1)
            
            # Robot label
            self.ax.text(x, y - 0.5, robot_name, 
                        ha='center', fontsize=8, fontweight='bold')
        
        # Add ROS2 info
        info_text = (
            f"📡 ROS2 Topics Active:\n"
            f"• /leader/odom, /follower_*/odom\n"
            f"• /leader/cmd_vel, /follower_*/cmd_vel\n"
            f"• /swarm/status\n"
            f"\n🎮 Try: ros2 topic list\n"
            f"⏱️ Time: {data['time']:.1f}s\n"
            f"🔧 Threading: FIXED ✅"
        )
        
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                    verticalalignment='top', fontsize=9,
                    bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    
    def stop(self):
        """Stop the visualization."""
        self.running = False

def main():
    print("🤖 Starting ROS2 Swarm Bridge Demo (Threading Fixed)...")
    print("🔄 This bridges matplotlib simulation to ROS2 topics")
    print("📡 Robot states published on ROS2 topics")
    print("🔧 Fixed: matplotlib threading issue on macOS")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create communication queue for visualization
    visualization_queue = queue.Queue(maxsize=100)
    
    try:
        # Create ROS2 node (runs in separate thread)
        node = SwarmControllerNode(visualization_queue)
        
        # Start ROS2 spinning in background thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        
        print("🚀 Demo running! Try these commands:")
        print("   ros2 topic list")
        print("   ros2 topic echo /swarm/status")
        print("   ros2 topic echo /leader/odom")
        print()
        
        # Run visualization in main thread (this is key!)
        visualization = SwarmVisualization(visualization_queue)
        visualization.run()
        
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    
    finally:
        # Clean shutdown
        if 'visualization' in locals():
            visualization.stop()
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Shutdown complete!")

if __name__ == '__main__':
    main() 