#!/usr/bin/env python3
"""
Unified ROS2 Swarm System Node

This node integrates all features from the Python demo:
- Formation Control: Triangle, Line, Circle, V-Shape
- Computer Vision: Leader detection and tracking
- Obstacle Avoidance: Static and dynamic obstacles
- Robot Collision Prevention: Prevents robots from passing through each other
- Multiple Controllers: Proportional and MPC
- Real-time Visualization: RViz markers and camera view

ROS2 Topics:
- /swarm/leader/pose - Leader position and orientation
- /swarm/followers/poses - Follower positions and orientations
- /swarm/formation_targets - Target positions for each follower
- /swarm/obstacles - Obstacle positions and properties
- /swarm/camera/image - Synthetic camera view
- /swarm/status - System status and parameters

Services:
- /swarm/set_formation - Change formation type
- /swarm/toggle_vision - Enable/disable vision system
- /swarm/toggle_obstacles - Enable/disable obstacle avoidance
- /swarm/add_obstacle - Add new obstacle
- /swarm/set_controller - Change controller type
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, Pose
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from typing import List, Dict, Tuple, Optional
import cv2
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Import our core logic
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../demos/python'))
from swarm_core import Robot, ProportionalController, get_formation_targets, VisionSystem, Obstacle, calculate_obstacle_avoidance, update_dynamic_obstacles, MultiRobotMPCController


class UnifiedSwarmROS2Node(Node):
    """
    Unified ROS2 Swarm System Node
    Integrates all features from the Python demo into ROS2
    """
    
    def __init__(self):
        super().__init__('unified_swarm_system')
        
        # Declare parameters directly
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('leader_speed', 1.2)
        self.declare_parameter('leader_radius', 8.0)
        self.declare_parameter('num_followers', 3)
        
        # Controller parameters
        self.declare_parameter('kp_distance', 2.5)
        self.declare_parameter('kp_angle', 4.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        
        # Collision avoidance parameters
        self.declare_parameter('min_robot_distance', 1.2)
        self.declare_parameter('obstacle_detection_range', 3.0)
        self.declare_parameter('obstacle_safety_distance', 1.5)
        
        # Vision parameters
        self.declare_parameter('camera_resolution', [640, 480])
        self.declare_parameter('camera_fov', [-10.0, 10.0, -10.0, 10.0])
        
        # Initialize system state
        self.initialize_system()
        
        # Setup ROS2 publishers and subscribers
        self.setup_ros_interface()
        
        # Setup static transform broadcaster for map frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.publish_static_transform()
        
        # Setup timers
        self.setup_timers()
        
        self.get_logger().info("🚀 Unified Swarm ROS2 System initialized!")
        self.get_logger().info("📡 Available services:")
        self.get_logger().info("   /swarm/set_formation")
        self.get_logger().info("   /swarm/toggle_vision")
        self.get_logger().info("   /swarm/toggle_obstacles")
        self.get_logger().info("   /swarm/add_obstacle")
        self.get_logger().info("   /swarm/set_controller")
    
    def initialize_system(self):
        """Initialize the swarm system"""
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.dt = 1.0 / self.update_rate
        self.time = 0.0
        
        # Initialize robots
        self.leader = Robot(0, 0, 0, robot_id=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(-2, -1.5, 0, robot_id=1, color='blue', marker='o', name='Follower_1'),
            Robot(-2, 1.5, 0, robot_id=2, color='green', marker='s', name='Follower_2'),
            Robot(-3, 0, 0, robot_id=3, color='orange', marker='^', name='Follower_3')
        ]
        
        # System state
        self.current_formation = "triangle"
        self.vision_enabled = False
        self.obstacles_enabled = True  # Enable obstacle avoidance by default
        self.robot_collision_avoidance = True
        
        # Controllers
        kp_distance = self.get_parameter('kp_distance').value
        kp_angle = self.get_parameter('kp_angle').value
        self.prop_controller = ProportionalController(kp_distance=kp_distance, kp_angle=kp_angle)
        self.mpc_controller = MultiRobotMPCController(num_robots=3, formation_type="triangle")
        self.controllers = ["Proportional", "MPC"]
        self.current_controller_idx = 0
        
        # Vision system
        self.vision_system = VisionSystem()
        self.camera_image = None
        self.cv_bridge = CvBridge()
        
        # Leader trajectory
        self.leader_speed = self.get_parameter('leader_speed').value
        self.leader_radius = self.get_parameter('leader_radius').value
        self.leader_center = np.array([0.0, 0.0])
        
        # Obstacles
        self.static_obstacles = [
            Obstacle(x=3, y=2, radius=1.0, color='darkred', name='Static_1'),
            Obstacle(x=-6, y=-3, radius=1.5, color='darkred', name='Static_2'),
            Obstacle(x=1, y=-4, radius=0.8, color='darkred', name='Static_3'),
            Obstacle(x=5, y=5, radius=1.2, color='darkred', name='Static_4'),
            Obstacle(x=-8, y=6, radius=1.0, color='darkred', name='Static_5')
        ]
        self.dynamic_obstacles = [
            Obstacle(x=-2, y=5, radius=0.8, vx=0.6, vy=-0.3, color='purple', is_dynamic=True, name='Dynamic_1'),
            Obstacle(x=6, y=-2, radius=1.0, vx=-0.45, vy=0.9, color='purple', is_dynamic=True, name='Dynamic_2'),
            Obstacle(x=7, y=7, radius=0.9, vx=-0.5, vy=-0.4, color='purple', is_dynamic=True, name='Dynamic_3'),
            Obstacle(x=-7, y=-7, radius=1.1, vx=0.4, vy=0.5, color='purple', is_dynamic=True, name='Dynamic_4')
        ]
        
        # Collision avoidance parameters
        self.min_robot_distance = self.get_parameter('min_robot_distance').value
        self.obstacle_detection_range = self.get_parameter('obstacle_detection_range').value
        self.obstacle_safety_distance = self.get_parameter('obstacle_safety_distance').value
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers, subscribers, and services"""
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.leader_pose_pub = self.create_publisher(
            PoseStamped, '/swarm/leader/pose', qos_profile
        )
        
        self.followers_poses_pub = self.create_publisher(
            PoseStamped, '/swarm/followers/poses', qos_profile
        )
        
        self.formation_targets_pub = self.create_publisher(
            Point, '/swarm/formation_targets', qos_profile
        )
        
        self.obstacles_pub = self.create_publisher(
            MarkerArray, '/swarm/obstacles', qos_profile
        )
        
        self.camera_image_pub = self.create_publisher(
            Image, '/swarm/camera/image', qos_profile
        )
        
        self.status_pub = self.create_publisher(
            String, '/swarm/status', qos_profile
        )
        
        self.robot_markers_pub = self.create_publisher(
            MarkerArray, '/swarm/robot_markers', qos_profile
        )
        
        # Services
        self.set_formation_srv = self.create_service(
            SetBool, '/swarm/set_formation', self.set_formation_callback
        )
        
        self.toggle_vision_srv = self.create_service(
            SetBool, '/swarm/toggle_vision', self.toggle_vision_callback
        )
        
        self.toggle_obstacles_srv = self.create_service(
            SetBool, '/swarm/toggle_obstacles', self.toggle_obstacles_callback
        )
        
        self.add_obstacle_srv = self.create_service(
            SetBool, '/swarm/add_obstacle', self.add_obstacle_callback
        )
        
        self.set_controller_srv = self.create_service(
            SetBool, '/swarm/set_controller', self.set_controller_callback
        )
    
    def setup_timers(self):
        """Setup ROS2 timers for periodic updates"""
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_system
        )
        
        self.publish_timer = self.create_timer(
            0.1, self.publish_data  # 10 Hz for publishing
        )
    
    def update_system(self):
        """Main system update loop"""
        self.update_leader()
        self.update_followers()
        
        # Update dynamic obstacles
        if self.obstacles_enabled:
            update_dynamic_obstacles(self.dynamic_obstacles, self.dt)
        
        self.publish_robot_transforms()
        self.time += self.dt
    
    def update_leader(self):
        """Update leader position"""
        # Leader always moves independently (circular trajectory)
        omega = self.leader_speed / self.leader_radius
        self.leader.x = self.leader_center[0] + self.leader_radius * math.cos(omega * self.time)
        self.leader.y = self.leader_center[1] + self.leader_radius * math.sin(omega * self.time)
        self.leader.theta = omega * self.time + math.pi/2
        
        # Apply obstacle avoidance for leader
        if self.obstacles_enabled:
            all_obstacles = self.static_obstacles + self.dynamic_obstacles
            avoidance_x, avoidance_y = calculate_obstacle_avoidance(self.leader, all_obstacles)
            self.leader.x += avoidance_x * self.dt * 0.5
            self.leader.y += avoidance_y * self.dt * 0.5
        
        # Apply robot-to-robot collision avoidance for leader
        if self.robot_collision_avoidance:
            leader_avoidance_x, leader_avoidance_y = self.calculate_leader_avoidance()
            self.leader.x += leader_avoidance_x * self.dt * 0.3
            self.leader.y += leader_avoidance_y * self.dt * 0.3
        
        # Update leader's internal state
        self.leader.update(self.dt)
        
        # Create camera image for vision system demonstration
        if self.vision_enabled:
            all_robots = [self.leader] + self.followers
            self.camera_image = self.vision_system.create_synthetic_image(all_robots)
    
    def update_followers(self):
        """Update follower positions"""
        leader_pos = np.array([self.leader.x, self.leader.y])
        formation_targets = get_formation_targets(leader_pos, self.current_formation)
        
        if self.controllers[self.current_controller_idx] == "Proportional":
            for i, follower in enumerate(self.followers):
                target_pos = formation_targets[i]
                control = self.prop_controller.compute_control(follower.get_state(), target_pos)
                
                # Apply obstacle avoidance if enabled
                if self.obstacles_enabled:
                    all_obstacles = self.static_obstacles + self.dynamic_obstacles
                    avoidance_x, avoidance_y = calculate_obstacle_avoidance(follower, all_obstacles)
                    follower.x += avoidance_x * self.dt * 0.5
                    follower.y += avoidance_y * self.dt * 0.5
                
                # Apply robot-to-robot collision avoidance
                if self.robot_collision_avoidance:
                    robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(follower, i)
                    follower.x += robot_avoidance_x * self.dt * 0.3
                    follower.y += robot_avoidance_y * self.dt * 0.3
                
                follower.update(self.dt, control[0], control[1])
        
        elif self.controllers[self.current_controller_idx] == "MPC":
            # Collect all robot states for MPC
            robot_states = [f.get_state() for f in self.followers]
            controls = self.mpc_controller.compute_control(robot_states, formation_targets)
            
            for i, (follower, control) in enumerate(zip(self.followers, controls)):
                # Apply obstacle avoidance if enabled
                if self.obstacles_enabled:
                    all_obstacles = self.static_obstacles + self.dynamic_obstacles
                    avoidance_x, avoidance_y = calculate_obstacle_avoidance(follower, all_obstacles)
                    follower.x += avoidance_x * self.dt * 0.5
                    follower.y += avoidance_y * self.dt * 0.5
                
                # Apply robot-to-robot collision avoidance
                if self.robot_collision_avoidance:
                    robot_avoidance_x, robot_avoidance_y = self.calculate_robot_avoidance(follower, i)
                    follower.x += robot_avoidance_x * self.dt * 0.3
                    follower.y += robot_avoidance_y * self.dt * 0.3
                
                follower.update(self.dt, control[0], control[1])
    
    def calculate_leader_avoidance(self):
        """Calculate avoidance forces to prevent leader from colliding with followers"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        for follower in self.followers:
            dx = self.leader.x - follower.x
            dy = self.leader.y - follower.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.min_robot_distance and distance > 0.1:
                force_magnitude = 2.0 * (self.min_robot_distance - distance) / distance
                avoidance_x += force_magnitude * dx
                avoidance_y += force_magnitude * dy
        
        # Limit avoidance force
        max_avoidance = 1.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y
    
    def calculate_robot_avoidance(self, current_robot, robot_index):
        """Calculate avoidance forces to prevent robot-to-robot collisions"""
        avoidance_x = 0.0
        avoidance_y = 0.0
        
        # Check collision with leader
        dx = current_robot.x - self.leader.x
        dy = current_robot.y - self.leader.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.min_robot_distance and distance > 0.1:
            force_magnitude = 2.0 * (self.min_robot_distance - distance) / distance
            avoidance_x += force_magnitude * dx
            avoidance_y += force_magnitude * dy
        
        # Check collision with other followers
        for j, other_robot in enumerate(self.followers):
            if j != robot_index:
                dx = current_robot.x - other_robot.x
                dy = current_robot.y - other_robot.y
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < self.min_robot_distance and distance > 0.1:
                    force_magnitude = 2.0 * (self.min_robot_distance - distance) / distance
                    avoidance_x += force_magnitude * dx
                    avoidance_y += force_magnitude * dy
        
        # Limit avoidance force
        max_avoidance = 1.5
        avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
        if avoidance_magnitude > max_avoidance:
            avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
            avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
        return avoidance_x, avoidance_y
    
    def publish_data(self):
        """Publish all ROS2 data"""
        self.publish_robot_poses()
        self.publish_formation_targets()
        self.publish_obstacles()
        self.publish_camera_image()
        self.publish_robot_markers()
        self.publish_status()
    
    def publish_robot_poses(self):
        """Publish robot poses"""
        # Publish leader pose
        leader_pose = PoseStamped()
        leader_pose.header.stamp = self.get_clock().now().to_msg()
        leader_pose.header.frame_id = "map"
        leader_pose.pose.position.x = float(self.leader.x)
        leader_pose.pose.position.y = float(self.leader.y)
        leader_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = self.euler_to_quaternion(0, 0, self.leader.theta)
        leader_pose.pose.orientation.x = float(q[0])
        leader_pose.pose.orientation.y = float(q[1])
        leader_pose.pose.orientation.z = float(q[2])
        leader_pose.pose.orientation.w = float(q[3])
        
        self.leader_pose_pub.publish(leader_pose)
        
        # Publish follower poses
        for follower in self.followers:
            follower_pose = PoseStamped()
            follower_pose.header.stamp = self.get_clock().now().to_msg()
            follower_pose.header.frame_id = "map"
            follower_pose.pose.position.x = float(follower.x)
            follower_pose.pose.position.y = float(follower.y)
            follower_pose.pose.position.z = 0.0
            
            q = self.euler_to_quaternion(0, 0, follower.theta)
            follower_pose.pose.orientation.x = float(q[0])
            follower_pose.pose.orientation.y = float(q[1])
            follower_pose.pose.orientation.z = float(q[2])
            follower_pose.pose.orientation.w = float(q[3])
            
            self.followers_poses_pub.publish(follower_pose)
    
    def publish_formation_targets(self):
        """Publish formation target positions"""
        leader_pos = np.array([self.leader.x, self.leader.y])
        formation_targets = get_formation_targets(leader_pos, self.current_formation)
        
        for i, target in enumerate(formation_targets):
            target_point = Point()
            target_point.x = float(target[0])
            target_point.y = float(target[1])
            target_point.z = 0.0
            self.formation_targets_pub.publish(target_point)
    
    def publish_obstacles(self):
        """Publish obstacle markers"""
        marker_array = MarkerArray()
        
        # Static obstacles
        for i, obstacle in enumerate(self.static_obstacles):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(obstacle.x)
            marker.pose.position.y = float(obstacle.y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = float(obstacle.radius * 2)
            marker.scale.y = float(obstacle.radius * 2)
            marker.scale.z = 0.1
            
            marker.color.r = 0.8
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            
            marker_array.markers.append(marker)
        
        # Dynamic obstacles
        for i, obstacle in enumerate(self.dynamic_obstacles):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.ns = "obstacles"
            marker.id = i + len(self.static_obstacles)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(obstacle.x)
            marker.pose.position.y = float(obstacle.y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = float(obstacle.radius * 2)
            marker.scale.y = float(obstacle.radius * 2)
            marker.scale.z = 0.1
            
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.5
            marker.color.a = 0.6
            
            marker_array.markers.append(marker)
        
        self.obstacles_pub.publish(marker_array)
    
    def publish_camera_image(self):
        """Publish camera image"""
        if self.camera_image is not None and self.vision_enabled:
            try:
                ros_image = self.cv_bridge.cv2_to_imgmsg(self.camera_image, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_frame"
                self.camera_image_pub.publish(ros_image)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish camera image: {e}")
    
    def publish_robot_markers(self):
        """Publish robot visualization markers"""
        marker_array = MarkerArray()
        
        # Leader marker
        leader_marker = Marker()
        leader_marker.header.stamp = self.get_clock().now().to_msg()
        leader_marker.header.frame_id = "map"
        leader_marker.ns = "robots"
        leader_marker.id = 0
        leader_marker.type = Marker.CYLINDER
        leader_marker.action = Marker.ADD
        
        leader_marker.pose.position.x = float(self.leader.x)
        leader_marker.pose.position.y = float(self.leader.y)
        leader_marker.pose.position.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.leader.theta)
        leader_marker.pose.orientation.x = float(q[0])
        leader_marker.pose.orientation.y = float(q[1])
        leader_marker.pose.orientation.z = float(q[2])
        leader_marker.pose.orientation.w = float(q[3])
        
        leader_marker.scale.x = 0.6
        leader_marker.scale.y = 0.6
        leader_marker.scale.z = 0.1
        
        leader_marker.color.r = 1.0
        leader_marker.color.g = 0.0
        leader_marker.color.b = 0.0
        leader_marker.color.a = 0.8
        
        marker_array.markers.append(leader_marker)
        
        # Follower markers
        colors = [(0, 0, 1), (0, 1, 0), (1, 0.5, 0)]  # Blue, Green, Orange
        for i, follower in enumerate(self.followers):
            follower_marker = Marker()
            follower_marker.header.stamp = self.get_clock().now().to_msg()
            follower_marker.header.frame_id = "map"
            follower_marker.ns = "robots"
            follower_marker.id = i + 1
            follower_marker.type = Marker.CYLINDER
            follower_marker.action = Marker.ADD
            
            follower_marker.pose.position.x = float(follower.x)
            follower_marker.pose.position.y = float(follower.y)
            follower_marker.pose.position.z = 0.0
            
            q = self.euler_to_quaternion(0, 0, follower.theta)
            follower_marker.pose.orientation.x = float(q[0])
            follower_marker.pose.orientation.y = float(q[1])
            follower_marker.pose.orientation.z = float(q[2])
            follower_marker.pose.orientation.w = float(q[3])
            
            follower_marker.scale.x = 0.5
            follower_marker.scale.y = 0.5
            follower_marker.scale.z = 0.1
            
            follower_marker.color.r = float(colors[i][0])
            follower_marker.color.g = float(colors[i][1])
            follower_marker.color.b = float(colors[i][2])
            follower_marker.color.a = 0.8
            
            marker_array.markers.append(follower_marker)
        
        self.robot_markers_pub.publish(marker_array)
    
    def publish_status(self):
        """Publish system status"""
        status_msg = String()
        status_msg.data = (
            f"Formation: {self.current_formation}, "
            f"Controller: {self.controllers[self.current_controller_idx]}, "
            f"Vision: {'ON' if self.vision_enabled else 'OFF'}, "
            f"Obstacles: {'ON' if self.obstacles_enabled else 'OFF'}, "
            f"Robot Collision: {'ON' if self.robot_collision_avoidance else 'OFF'}, "
            f"Time: {self.time:.1f}s"
        )
        self.status_pub.publish(status_msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr
        
        return q
    
    def publish_robot_transforms(self):
        # Publish map -> leader/base_link
        self._publish_robot_tf(self.leader, 'leader')
        for i, follower in enumerate(self.followers, 1):
            self._publish_robot_tf(follower, f'follower_{i}')

    def _publish_robot_tf(self, robot, ns):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{ns}/base_link'
        t.transform.translation.x = robot.x
        t.transform.translation.y = robot.y
        t.transform.translation.z = 0.0
        # Convert theta to quaternion (yaw only, roll=pitch=0)
        yaw = robot.theta
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
    
    # Service callbacks
    def set_formation_callback(self, request, response):
        """Service callback to change formation"""
        formations = ["triangle", "line", "circle", "v_shape"]
        if request.data:
            # Cycle through formations
            current_idx = formations.index(self.current_formation)
            self.current_formation = formations[(current_idx + 1) % len(formations)]
            self.get_logger().info(f"🔄 Formation changed to: {self.current_formation}")
            response.success = True
            response.message = f"Formation set to {self.current_formation}"
        else:
            response.success = False
            response.message = f"Invalid request. Available formations: {formations}"
        return response
    
    def toggle_vision_callback(self, request, response):
        """Service callback to toggle vision"""
        self.vision_enabled = request.data
        self.get_logger().info(f"👁️ Vision {'enabled' if self.vision_enabled else 'disabled'}")
        response.success = True
        response.message = f"Vision {'enabled' if self.vision_enabled else 'disabled'}"
        return response
    
    def toggle_obstacles_callback(self, request, response):
        """Service callback to toggle obstacles"""
        self.obstacles_enabled = request.data
        self.get_logger().info(f"🚧 Obstacles {'enabled' if self.obstacles_enabled else 'disabled'}")
        response.success = True
        response.message = f"Obstacles {'enabled' if self.obstacles_enabled else 'disabled'}"
        return response
    
    def add_obstacle_callback(self, request, response):
        """Service callback to add obstacle"""
        if request.data:
            # Add a random obstacle
            import random
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            radius = random.uniform(0.5, 1.5)
            if random.random() < 0.7:
                self.static_obstacles.append(Obstacle(x, y, radius, color='darkred'))
            else:
                vx = random.uniform(-0.8, 0.8)
                vy = random.uniform(-0.8, 0.8)
                self.dynamic_obstacles.append(Obstacle(x, y, radius, vx, vy, color='purple', is_dynamic=True))
            
            self.get_logger().info(f"🚧 Added obstacle at ({x:.1f}, {y:.1f})")
            response.success = True
            response.message = f"Added obstacle at ({x:.1f}, {y:.1f})"
        else:
            response.success = False
            response.message = "Invalid request"
        return response
    
    def set_controller_callback(self, request, response):
        """Service callback to change controller"""
        if request.data:
            # Cycle through controllers
            self.current_controller_idx = (self.current_controller_idx + 1) % len(self.controllers)
            controller_name = self.controllers[self.current_controller_idx]
            self.get_logger().info(f"🎛️ Controller switched to: {controller_name}")
            response.success = True
            response.message = f"Controller switched to {controller_name}"
        else:
            response.success = False
            response.message = "Invalid request"
        return response

    def publish_static_transform(self):
        """Publish static transform for map frame"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        
        # Identity transform (map and base_link are the same)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published static transform for map frame")


def main(args=None):
    rclpy.init(args=args)
    
    node = UnifiedSwarmROS2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 