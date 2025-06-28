#!/usr/bin/env python3
"""
🎥 Vision-Based Leader-Follower ROS2 System
Complete ROS2 implementation with separate nodes for vision and control
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional
from matplotlib.animation import FuncAnimation

# ROS2 imports
from geometry_msgs.msg import Twist, Point, Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge
import rclpy.logging

@dataclass
class Robot:
    """Robot state representation"""
    x: float
    y: float
    theta: float
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    color: str = 'blue'
    marker: str = 'o'
    name: str = 'robot'

class VisionSystem:
    """Simulated camera vision system for robot detection"""
    
    def __init__(self, camera_resolution=(640, 480), field_of_view=(-10, 10, -10, 10)):
        self.resolution = camera_resolution
        self.fov = field_of_view
        
        # Color ranges for robot detection (HSV)
        self.color_ranges = {
            'red': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
            'blue': {'lower': np.array([100, 100, 100]), 'upper': np.array([130, 255, 255])},
            'green': {'lower': np.array([40, 100, 100]), 'upper': np.array([80, 255, 255])},
            'orange': {'lower': np.array([10, 100, 100]), 'upper': np.array([25, 255, 255])}
        }
        
        # Detection history for smoothing
        self.detection_history = []
        self.max_history = 5
        
    def world_to_camera(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """Convert world coordinates to camera pixel coordinates"""
        cam_x = (world_x - self.fov[0]) / (self.fov[1] - self.fov[0])
        cam_y = (world_y - self.fov[2]) / (self.fov[3] - self.fov[2])
        
        pixel_x = int(cam_x * self.resolution[0])
        pixel_y = int((1 - cam_y) * self.resolution[1])
        
        return pixel_x, pixel_y
    
    def camera_to_world(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """Convert camera pixel coordinates to world coordinates"""
        cam_x = pixel_x / self.resolution[0]
        cam_y = 1 - (pixel_y / self.resolution[1])
        
        world_x = cam_x * (self.fov[1] - self.fov[0]) + self.fov[0]
        world_y = cam_y * (self.fov[3] - self.fov[2]) + self.fov[2]
        
        return world_x, world_y
    
    def create_synthetic_image(self, robots: List[Robot]) -> np.ndarray:
        """Create a synthetic camera image with robots"""
        image = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        image[:] = [50, 50, 50]
        
        for robot in robots:
            if (self.fov[0] <= robot.x <= self.fov[1] and 
                self.fov[2] <= robot.y <= self.fov[3]):
                
                pixel_x, pixel_y = self.world_to_camera(robot.x, robot.y)
                
                color_map = {
                    'red': (0, 0, 255), 'blue': (255, 0, 0), 
                    'green': (0, 255, 0), 'orange': (0, 165, 255)
                }
                color = color_map.get(robot.color, (255, 255, 255))
                
                cv2.circle(image, (pixel_x, pixel_y), 15, color, -1)
                
                end_x = pixel_x + int(20 * np.cos(robot.theta))
                end_y = pixel_y - int(20 * np.sin(robot.theta))
                cv2.arrowedLine(image, (pixel_x, pixel_y), (end_x, end_y), (255, 255, 255), 2)
        
        return image
    
    def detect_leader(self, image: np.ndarray, target_color: str = 'red') -> Optional[Tuple[float, float]]:
        """Detect leader robot using color-based detection"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if target_color not in self.color_ranges:
            return None
            
        color_range = self.color_ranges[target_color]
        mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
            
        pixel_x = int(M["m10"] / M["m00"])
        pixel_y = int(M["m01"] / M["m00"])
        
        world_x, world_y = self.camera_to_world(pixel_x, pixel_y)
        
        self.detection_history.append((world_x, world_y))
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
        
        if len(self.detection_history) >= 3:
            avg_x = np.mean([pos[0] for pos in self.detection_history])
            avg_y = np.mean([pos[1] for pos in self.detection_history])
            return avg_x, avg_y
        
        return world_x, world_y

class VisualizationNode(Node):
    """ROS2 Node for matplotlib visualization"""
    
    def __init__(self):
        super().__init__('visualization_node')
        
        # Robot states for visualization
        self.leader = Robot(x=0, y=0, theta=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(x=-2, y=-2, theta=0, color='blue', marker='o', name='Follower_1'),
            Robot(x=-2, y=2, theta=0, color='green', marker='s', name='Follower_2'),
            Robot(x=-4, y=0, theta=0, color='orange', marker='^', name='Follower_3')
        ]
        
        # Visualization setup
        self.setup_visualization()
        
        # Trail tracking
        self.trails = {robot.name: {'x': [], 'y': []} for robot in [self.leader] + self.followers}
        self.max_trail_length = 100
        
        # Vision system for camera view
        self.vision = VisionSystem()
        self.camera_image = None
        self.detection_status = False
        self.detected_position = None
        
        # Subscribers
        self.leader_pose_sub = self.create_subscription(
            Pose2D, '/leader_pose', self.leader_pose_callback, 10)
        
        for i in range(3):
            sub = self.create_subscription(
                Pose2D, f'/follower_{i+1}_pose',
                lambda msg, idx=i: self.follower_pose_callback(msg, idx), 10)
        
        self.camera_image_sub = self.create_subscription(
            Image, '/camera_image', self.camera_image_callback, 10)
        self.detection_status_sub = self.create_subscription(
            Bool, '/detection_status', self.detection_status_callback, 10)
        self.detected_pose_sub = self.create_subscription(
            Pose2D, '/leader_detected_pose', self.detected_pose_callback, 10)
        
        # Animation timer
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        self.get_logger().info('🎨 Visualization Node Started!')
        
    def setup_visualization(self):
        """Setup matplotlib visualization"""
        plt.style.use('dark_background')
        self.fig, (self.ax_main, self.ax_camera) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main simulation view
        self.ax_main.set_xlim(-12, 12)
        self.ax_main.set_ylim(-8, 8)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_title('🎥 Vision-Based ROS2 Multi-Robot System\n🔴 Leader | 🔵🟢🟠 Followers | 🎮 Formation Control', 
                              fontsize=14, fontweight='bold')
        self.ax_main.set_xlabel('X Position (m)')
        self.ax_main.set_ylabel('Y Position (m)')
        
        # Camera view
        self.ax_camera.set_title('📹 Synthetic Camera View\n(Leader Detection)', fontsize=12, fontweight='bold')
        self.ax_camera.set_xticks([])
        self.ax_camera.set_yticks([])
        
        # Initialize empty plots
        self.robot_plots = {}
        self.trail_plots = {}
        
        # Create robot plots
        colors = ['red', 'blue', 'green', 'orange']
        markers = ['*', 'o', 's', '^']
        names = ['Leader', 'Follower_1', 'Follower_2', 'Follower_3']
        
        for i, (color, marker, name) in enumerate(zip(colors, markers, names)):
            # Robot position
            self.robot_plots[name] = self.ax_main.scatter([], [], c=color, marker=marker, 
                                                         s=150, edgecolors='white', linewidth=2,
                                                         label=name)
            
            # Trail
            self.trail_plots[name], = self.ax_main.plot([], [], color=color, alpha=0.3, linewidth=1)
        
        # Detection indicator
        self.detection_plot = self.ax_main.scatter([], [], c='yellow', marker='X', s=200, 
                                                  edgecolors='black', linewidth=2, 
                                                  label='Vision Detection')
        
        self.ax_main.legend(loc='upper left', bbox_to_anchor=(0, 1))
        
        # Status text
        self.status_text = self.ax_main.text(0.02, 0.98, '', transform=self.ax_main.transAxes, 
                                           fontsize=10, verticalalignment='top',
                                           bbox=dict(boxstyle='round', facecolor='black', alpha=0.8))
        
    def leader_pose_callback(self, msg):
        """Update leader pose"""
        self.leader.x = msg.x
        self.leader.y = msg.y
        self.leader.theta = msg.theta
        
    def follower_pose_callback(self, msg, follower_id):
        """Update follower pose"""
        if follower_id < len(self.followers):
            self.followers[follower_id].x = msg.x
            self.followers[follower_id].y = msg.y
            self.followers[follower_id].theta = msg.theta
    
    def camera_image_callback(self, msg):
        """Update camera image"""
        try:
            bridge = CvBridge()
            self.camera_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Camera image conversion failed: {e}')
    
    def detection_status_callback(self, msg):
        """Update detection status"""
        self.detection_status = msg.data
        
    def detected_pose_callback(self, msg):
        """Update detected leader position"""
        self.detected_position = (msg.x, msg.y)
    
    def update_trails(self):
        """Update robot trails"""
        all_robots = [self.leader] + self.followers
        
        for robot in all_robots:
            trail = self.trails[robot.name]
            trail['x'].append(robot.x)
            trail['y'].append(robot.y)
            
            # Limit trail length
            if len(trail['x']) > self.max_trail_length:
                trail['x'].pop(0)
                trail['y'].pop(0)
    
    def update_plot(self, frame):
        """Update visualization plots"""
        try:
            # Update trails
            self.update_trails()
            
            # Update robot positions
            all_robots = [self.leader] + self.followers
            
            for robot in all_robots:
                # Update robot position
                self.robot_plots[robot.name].set_offsets([[robot.x, robot.y]])
                
                # Update trail
                trail = self.trails[robot.name]
                self.trail_plots[robot.name].set_data(trail['x'], trail['y'])
            
            # Update detection indicator
            if self.detection_status and self.detected_position:
                self.detection_plot.set_offsets([[self.detected_position[0], self.detected_position[1]]])
            else:
                self.detection_plot.set_offsets([[]])
            
            # Update camera view
            if self.camera_image is not None:
                self.ax_camera.clear()
                self.ax_camera.imshow(cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB))
                self.ax_camera.set_title('📹 Synthetic Camera View\n(Leader Detection)', 
                                       fontsize=12, fontweight='bold')
                self.ax_camera.set_xticks([])
                self.ax_camera.set_yticks([])
                
                # Add detection status overlay
                status_color = 'green' if self.detection_status else 'red'
                status_text = 'DETECTED' if self.detection_status else 'SEARCHING'
                self.ax_camera.text(10, 30, f'Vision: {status_text}', 
                                  color=status_color, fontsize=12, fontweight='bold')
            
            # Update status text
            status_msg = f"""🎥 Vision-Based ROS2 System Status:
🔴 Leader: ({self.leader.x:.1f}, {self.leader.y:.1f})
📹 Vision: {'ACTIVE' if self.detection_status else 'INACTIVE'}
🎯 Detection: {'YES' if self.detected_position else 'NO'}
🤖 ROS2 Nodes: Leader, Vision, Controller, 3x Followers"""
            
            self.status_text.set_text(status_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Visualization update failed: {e}')

class LeaderRobotNode(Node):
    """ROS2 Leader Robot Node"""
    
    def __init__(self):
        super().__init__('leader_robot')
        
        # Robot state
        self.robot = Robot(x=0, y=0, theta=0, color='red', marker='*', name='Leader')
        self.leader_speed = 0.3
        
        # Publishers
        self.pose_publisher = self.create_publisher(Pose2D, '/leader_pose', 10)
        
        # Timer for motion updates
        self.timer = self.create_timer(0.05, self.update_motion)
        
        self.get_logger().info('🔴 Leader Robot Node Started!')
        
    def update_motion(self):
        """Update leader motion and publish pose"""
        t = time.time()
        
        # Circular motion
        radius = 3.0
        self.robot.x = radius * np.cos(self.leader_speed * t)
        self.robot.y = radius * np.sin(self.leader_speed * t)
        self.robot.theta = self.leader_speed * t + np.pi/2
        
        # Publish pose
        pose_msg = Pose2D()
        pose_msg.x = self.robot.x
        pose_msg.y = self.robot.y
        pose_msg.theta = self.robot.theta
        self.pose_publisher.publish(pose_msg)

class VisionNode(Node):
    """ROS2 Vision Processing Node"""
    
    def __init__(self):
        super().__init__('vision_node')
        
        # Vision system
        self.vision = VisionSystem()
        self.bridge = CvBridge()
        self.vision_enabled = True
        
        # Robot states (for simulation)
        self.leader = Robot(x=0, y=0, theta=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(x=-2, y=-2, theta=0, color='blue', marker='o', name='Follower_1'),
            Robot(x=-2, y=2, theta=0, color='green', marker='s', name='Follower_2'),
            Robot(x=-4, y=0, theta=0, color='orange', marker='^', name='Follower_3')
        ]
        
        # Subscribers
        self.leader_pose_sub = self.create_subscription(
            Pose2D, '/leader_pose', self.leader_pose_callback, 10)
        self.vision_toggle_sub = self.create_subscription(
            Bool, '/vision_toggle', self.vision_toggle_callback, 10)
        
        # Publishers
        self.detected_pose_pub = self.create_publisher(Pose2D, '/leader_detected_pose', 10)
        self.camera_image_pub = self.create_publisher(Image, '/camera_image', 10)
        self.detection_status_pub = self.create_publisher(Bool, '/detection_status', 10)
        
        # Timer for vision processing
        self.timer = self.create_timer(0.1, self.process_vision)
        
        self.get_logger().info('📹 Vision Node Started!')
        
    def leader_pose_callback(self, msg):
        """Update leader pose from leader node"""
        self.leader.x = msg.x
        self.leader.y = msg.y
        self.leader.theta = msg.theta
        
    def vision_toggle_callback(self, msg):
        """Toggle vision system on/off"""
        self.vision_enabled = msg.data
        status = "ON" if self.vision_enabled else "OFF"
        self.get_logger().info(f'🎥 Vision system: {status}')
        
    def process_vision(self):
        """Process vision and detect leader"""
        if not self.vision_enabled:
            return
            
        # Create synthetic camera image
        all_robots = [self.leader] + self.followers
        camera_image = self.vision.create_synthetic_image(all_robots)
        
        # Publish camera image
        image_msg = self.bridge.cv2_to_imgmsg(camera_image, encoding='bgr8')
        self.camera_image_pub.publish(image_msg)
        
        # Detect leader robot
        detected_pos = self.vision.detect_leader(camera_image, target_color='red')
        
        if detected_pos is not None:
            # Publish detected pose
            pose_msg = Pose2D()
            pose_msg.x = detected_pos[0]
            pose_msg.y = detected_pos[1]
            pose_msg.theta = self.leader.theta  # Use known orientation
            self.detected_pose_pub.publish(pose_msg)
            
            # Publish detection status
            status_msg = Bool()
            status_msg.data = True
            self.detection_status_pub.publish(status_msg)
        else:
            # Publish detection failure
            status_msg = Bool()
            status_msg.data = False
            self.detection_status_pub.publish(status_msg)

class FollowerRobotNode(Node):
    """ROS2 Follower Robot Node"""
    
    def __init__(self, robot_id: int):
        super().__init__(f'follower_robot_{robot_id}')
        
        self.robot_id = robot_id
        self.robot = Robot(x=-2-robot_id, y=(-1)**robot_id * 2, theta=0, 
                          color=['blue', 'green', 'orange'][robot_id], 
                          marker=['o', 's', '^'][robot_id], 
                          name=f'Follower_{robot_id+1}')
        
        # Control parameters
        self.dt = 0.05
        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.5
        self.use_vision = True
        
        # Formation parameters
        self.formations = {
            'triangle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'line': [(-2.0, 0.0), (-4.0, 0.0), (-6.0, 0.0)],
            'circle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'v_shape': [(-2.5, -2.0), (-2.5, 2.0), (-4.5, 0.0)]
        }
        self.current_formation = 'triangle'
        
        # Leader tracking
        self.leader_pose = None
        self.detected_leader_pose = None
        self.detection_active = False
        
        # Subscribers
        self.leader_pose_sub = self.create_subscription(
            Pose2D, '/leader_pose', self.leader_pose_callback, 10)
        self.detected_pose_sub = self.create_subscription(
            Pose2D, '/leader_detected_pose', self.detected_pose_callback, 10)
        self.detection_status_sub = self.create_subscription(
            Bool, '/detection_status', self.detection_status_callback, 10)
        self.formation_sub = self.create_subscription(
            String, '/formation_command', self.formation_callback, 10)
        
        # Publishers
        self.pose_publisher = self.create_publisher(Pose2D, f'/follower_{robot_id+1}_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/follower_{robot_id+1}/cmd_vel', 10)
        
        # Timer for control updates
        self.timer = self.create_timer(self.dt, self.update_control)
        
        self.get_logger().info(f'🔵 Follower Robot {robot_id+1} Started!')
        
    def leader_pose_callback(self, msg):
        """Update leader pose"""
        self.leader_pose = msg
        
    def detected_pose_callback(self, msg):
        """Update detected leader pose from vision"""
        self.detected_leader_pose = msg
        
    def detection_status_callback(self, msg):
        """Update detection status"""
        self.detection_active = msg.data
        
    def formation_callback(self, msg):
        """Update formation command"""
        if msg.data in self.formations:
            self.current_formation = msg.data
            self.get_logger().info(f'🔄 Switched to {msg.data.upper()} formation')
    
    def get_formation_target(self) -> Tuple[float, float]:
        """Get target position for this follower in current formation"""
        # Choose leader pose source
        if self.use_vision and self.detection_active and self.detected_leader_pose:
            leader_x, leader_y = self.detected_leader_pose.x, self.detected_leader_pose.y
            leader_theta = self.detected_leader_pose.theta
        elif self.leader_pose:
            leader_x, leader_y = self.leader_pose.x, self.leader_pose.y
            leader_theta = self.leader_pose.theta
        else:
            return self.robot.x, self.robot.y  # Stay in place
        
        # Get formation offset
        formation_offsets = self.formations[self.current_formation]
        if self.robot_id >= len(formation_offsets):
            return leader_x - 2.0, leader_y
            
        dx, dy = formation_offsets[self.robot_id]
        
        # Rotate offset based on leader orientation
        cos_theta = np.cos(leader_theta)
        sin_theta = np.sin(leader_theta)
        
        target_x = leader_x + dx * cos_theta - dy * sin_theta
        target_y = leader_y + dx * sin_theta + dy * cos_theta
        
        return target_x, target_y
    
    def update_control(self):
        """Update follower control"""
        target_x, target_y = self.get_formation_target()
        
        # Calculate error
        error_x = target_x - self.robot.x
        error_y = target_y - self.robot.y  
        distance_error = np.sqrt(error_x**2 + error_y**2)
        
        if distance_error < 0.01:
            self.robot.vx = self.robot.vy = self.robot.omega = 0.0
        else:
            # Proportional control
            kp_linear = 0.3
            kp_angular = 1.0
            
            # Calculate desired velocities
            desired_vx = kp_linear * error_x
            desired_vy = kp_linear * error_y
            
            # Limit velocities
            desired_vx = np.clip(desired_vx, -self.max_linear_vel, self.max_linear_vel)
            desired_vy = np.clip(desired_vy, -self.max_linear_vel, self.max_linear_vel)
            
            # Update robot state
            self.robot.vx = desired_vx
            self.robot.vy = desired_vy
            
            # Update position
            self.robot.x += self.robot.vx * self.dt
            self.robot.y += self.robot.vy * self.dt
            
            # Update orientation
            if abs(self.robot.vx) > 0.1 or abs(self.robot.vy) > 0.1:
                desired_theta = np.arctan2(self.robot.vy, self.robot.vx)
                theta_error = desired_theta - self.robot.theta
                
                # Normalize angle
                while theta_error > np.pi:
                    theta_error -= 2 * np.pi
                while theta_error < -np.pi:
                    theta_error += 2 * np.pi
                    
                self.robot.omega = kp_angular * theta_error
                self.robot.omega = np.clip(self.robot.omega, -self.max_angular_vel, self.max_angular_vel)
                self.robot.theta += self.robot.omega * self.dt
        
        # Publish pose
        pose_msg = Pose2D()
        pose_msg.x = self.robot.x
        pose_msg.y = self.robot.y
        pose_msg.theta = self.robot.theta
        self.pose_publisher.publish(pose_msg)
        
        # Publish cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(np.sqrt(self.robot.vx**2 + self.robot.vy**2))
        cmd_vel_msg.angular.z = float(self.robot.omega)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

class ControllerNode(Node):
    """ROS2 Main Controller Node"""
    
    def __init__(self):
        super().__init__('controller_node')
        
        # Publishers
        self.formation_pub = self.create_publisher(String, '/formation_command', 10)
        self.vision_toggle_pub = self.create_publisher(Bool, '/vision_toggle', 10)
        
        # Formation control
        self.formations = ['triangle', 'line', 'circle', 'v_shape']
        self.formation_index = 0
        
        self.get_logger().info('🎮 Controller Node Started!')
        self.get_logger().info('Commands: "formation" to switch, "vision" to toggle')
        
        # Timer for user input (simplified for demo)
        self.timer = self.create_timer(5.0, self.cycle_formation)
        
    def cycle_formation(self):
        """Automatically cycle through formations for demo"""
        formation = self.formations[self.formation_index]
        
        msg = String()
        msg.data = formation
        self.formation_pub.publish(msg)
        
        self.get_logger().info(f'🔄 Switching to {formation.upper()} formation')
        
        self.formation_index = (self.formation_index + 1) % len(self.formations)

def main():
    """Main function to run all ROS2 nodes"""
    rclpy.init()
    
    try:
        # Create nodes
        leader_node = LeaderRobotNode()
        vision_node = VisionNode()
        controller_node = ControllerNode()
        visualization_node = VisualizationNode()
        
        # Create follower nodes
        follower_nodes = []
        for i in range(3):
            follower_nodes.append(FollowerRobotNode(i))
        
        # Create executor and add nodes
        executor = MultiThreadedExecutor()
        executor.add_node(leader_node)
        executor.add_node(vision_node)
        executor.add_node(controller_node)
        executor.add_node(visualization_node)
        for follower in follower_nodes:
            executor.add_node(follower)
        
        print("🚀 Starting Vision-Based ROS2 Multi-Robot System...")
        print("🔴 Leader publishing to /leader_pose")
        print("📹 Vision node processing and publishing to /leader_detected_pose")
        print("🔵🟢🟠 Followers subscribing and following formation")
        print("🎮 Controller cycling through formations every 5 seconds")
        print("🎨 Visualization showing robot movements and camera view")
        print("📡 Topics: /leader_pose, /leader_detected_pose, /formation_command, /vision_toggle")
        print("Press Ctrl+C to stop")
        
        # Start visualization in separate thread
        def run_visualization():
            plt.show()
        
        viz_thread = threading.Thread(target=run_visualization)
        viz_thread.daemon = True
        viz_thread.start()
        
        # Spin executor
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping ROS2 nodes...")
    finally:
        # Cleanup
        leader_node.destroy_node()
        vision_node.destroy_node()
        controller_node.destroy_node()
        visualization_node.destroy_node()
        for follower in follower_nodes:
            follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 