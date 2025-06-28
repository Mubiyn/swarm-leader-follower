#!/usr/bin/env python3
"""
🎥 Vision-Based Leader-Follower Demo
Replaces known leader position with camera-based detection
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional
from matplotlib.animation import FuncAnimation

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
        self.fov = field_of_view  # (x_min, x_max, y_min, y_max) in world coordinates
        
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
        pixel_y = int((1 - cam_y) * self.resolution[1])  # Flip Y axis
        
        return pixel_x, pixel_y
    
    def camera_to_world(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """Convert camera pixel coordinates to world coordinates"""
        cam_x = pixel_x / self.resolution[0]
        cam_y = 1 - (pixel_y / self.resolution[1])  # Flip Y axis
        
        world_x = cam_x * (self.fov[1] - self.fov[0]) + self.fov[0]
        world_y = cam_y * (self.fov[3] - self.fov[2]) + self.fov[2]
        
        return world_x, world_y
    
    def create_synthetic_image(self, robots: List[Robot]) -> np.ndarray:
        """Create a synthetic camera image with robots"""
        image = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        image[:] = [50, 50, 50]  # Dark gray background
        
        # Draw robots on the image
        for robot in robots:
            if (self.fov[0] <= robot.x <= self.fov[1] and 
                self.fov[2] <= robot.y <= self.fov[3]):
                
                pixel_x, pixel_y = self.world_to_camera(robot.x, robot.y)
                
                # Draw robot as colored circle
                color_map = {
                    'red': (0, 0, 255), 'blue': (255, 0, 0), 
                    'green': (0, 255, 0), 'orange': (0, 165, 255)
                }
                color = color_map.get(robot.color, (255, 255, 255))
                
                cv2.circle(image, (pixel_x, pixel_y), 15, color, -1)
                
                # Add robot orientation indicator
                end_x = pixel_x + int(20 * np.cos(robot.theta))
                end_y = pixel_y - int(20 * np.sin(robot.theta))
                cv2.arrowedLine(image, (pixel_x, pixel_y), (end_x, end_y), (255, 255, 255), 2)
        
        return image
    
    def detect_leader(self, image: np.ndarray, target_color: str = 'red') -> Optional[Tuple[float, float]]:
        """Detect leader robot using color-based detection"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if target_color not in self.color_ranges:
            return None
            
        color_range = self.color_ranges[target_color]
        
        # Create mask for target color
        mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        # Find largest contour (assumed to be the leader)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
            
        pixel_x = int(M["m10"] / M["m00"])
        pixel_y = int(M["m01"] / M["m00"])
        
        # Convert back to world coordinates
        world_x, world_y = self.camera_to_world(pixel_x, pixel_y)
        
        # Add to detection history for smoothing
        self.detection_history.append((world_x, world_y))
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
        
        # Return smoothed position
        if len(self.detection_history) >= 3:
            avg_x = np.mean([pos[0] for pos in self.detection_history])
            avg_y = np.mean([pos[1] for pos in self.detection_history])
            return avg_x, avg_y
        
        return world_x, world_y

class VisionBasedSwarmController:
    """Vision-based multi-robot swarm controller"""
    
    def __init__(self):
        # Initialize robots
        self.leader = Robot(x=0, y=0, theta=0, color='red', marker='*', name='Leader')
        self.followers = [
            Robot(x=-2, y=-2, theta=0, color='blue', marker='o', name='Follower_1'),
            Robot(x=-2, y=2, theta=0, color='green', marker='s', name='Follower_2'),
            Robot(x=-4, y=0, theta=0, color='orange', marker='^', name='Follower_3')
        ]
        
        # Vision system
        self.vision = VisionSystem()
        self.detected_leader_pos = None
        self.vision_enabled = True
        self.current_camera_image = None
        
        # Control parameters
        self.dt = 0.05
        self.leader_speed = 0.3
        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.5
        
        # Formation parameters
        self.formations = {
            'triangle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'line': [(-2.0, 0.0), (-4.0, 0.0), (-6.0, 0.0)],
            'circle': [(-2.5, -1.5), (-2.5, 1.5), (-4.0, 0.0)],
            'v_shape': [(-2.5, -2.0), (-2.5, 2.0), (-4.5, 0.0)]
        }
        self.current_formation = 'triangle'
        self.formation_keys = list(self.formations.keys())
        self.formation_index = 0
        
        # Visualization
        self.setup_visualization()
        
        # Control thread
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        
    def setup_visualization(self):
        """Setup matplotlib visualization"""
        self.fig, (self.ax_main, self.ax_camera) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main simulation view
        self.ax_main.set_xlim(-12, 4)
        self.ax_main.set_ylim(-8, 8)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_title('🎥 Vision-Based Leader-Follower System', fontsize=14, fontweight='bold')
        
        # Camera view
        self.ax_camera.set_xlim(0, 640)
        self.ax_camera.set_ylim(0, 480)
        self.ax_camera.set_aspect('equal')
        self.ax_camera.set_title('📹 Camera View (Robot Detection)', fontsize=12)
        self.ax_camera.invert_yaxis()  # Match image coordinates
        
        # Initialize trails
        self.trails = {robot.name: {'x': [], 'y': []} for robot in [self.leader] + self.followers}
        
        plt.tight_layout()
        
    def update_leader_motion(self):
        """Update leader robot motion (circular path)"""
        t = time.time()
        
        # Circular motion
        radius = 3.0
        self.leader.x = radius * np.cos(self.leader_speed * t)
        self.leader.y = radius * np.sin(self.leader_speed * t)
        self.leader.theta = self.leader_speed * t + np.pi/2
        
    def get_formation_target(self, follower_idx: int) -> Tuple[float, float]:
        """Get target position for follower in current formation"""
        if self.detected_leader_pos is None:
            # Fallback to known leader position
            leader_x, leader_y = self.leader.x, self.leader.y
            leader_theta = self.leader.theta
        else:
            # Use detected leader position
            leader_x, leader_y = self.detected_leader_pos
            leader_theta = self.leader.theta  # Still use known orientation for formation
        
        # Get formation offset
        formation_offsets = self.formations[self.current_formation]
        if follower_idx >= len(formation_offsets):
            return leader_x - 2.0, leader_y
            
        dx, dy = formation_offsets[follower_idx]
        
        # Rotate offset based on leader orientation
        cos_theta = np.cos(leader_theta)
        sin_theta = np.sin(leader_theta)
        
        target_x = leader_x + dx * cos_theta - dy * sin_theta
        target_y = leader_y + dx * sin_theta + dy * cos_theta
        
        return target_x, target_y
    
    def update_follower_control(self, follower: Robot, target_x: float, target_y: float):
        """Update follower control using proportional controller"""
        # Calculate error
        error_x = target_x - follower.x
        error_y = target_y - follower.y  
        distance_error = np.sqrt(error_x**2 + error_y**2)
        
        # Avoid division by zero
        if distance_error < 0.01:
            follower.vx = follower.vy = follower.omega = 0.0
            return
            
        # Proportional control gains
        kp_linear = 0.3
        kp_angular = 1.0
        
        # Calculate desired velocities
        desired_vx = kp_linear * error_x
        desired_vy = kp_linear * error_y
        
        # Limit velocities
        desired_vx = np.clip(desired_vx, -self.max_linear_vel, self.max_linear_vel)
        desired_vy = np.clip(desired_vy, -self.max_linear_vel, self.max_linear_vel)
        
        # Update robot state
        follower.vx = desired_vx
        follower.vy = desired_vy
        
        # Update position
        follower.x += follower.vx * self.dt
        follower.y += follower.vy * self.dt
        
        # Update orientation to face movement direction
        if abs(follower.vx) > 0.1 or abs(follower.vy) > 0.1:
            desired_theta = np.arctan2(follower.vy, follower.vx)
            theta_error = desired_theta - follower.theta
            
            # Normalize angle
            while theta_error > np.pi:
                theta_error -= 2 * np.pi
            while theta_error < -np.pi:
                theta_error += 2 * np.pi
                
            follower.omega = kp_angular * theta_error
            follower.omega = np.clip(follower.omega, -self.max_angular_vel, self.max_angular_vel)
            follower.theta += follower.omega * self.dt
    
    def collision_avoidance(self, robot: Robot, all_robots: List[Robot]):
        """Simple collision avoidance between robots"""
        avoidance_force_x = 0.0
        avoidance_force_y = 0.0
        min_distance = 1.0  # Minimum safe distance
        
        for other in all_robots:
            if other.name == robot.name:
                continue
                
            # Calculate distance
            dx = robot.x - other.x
            dy = robot.y - other.y
            distance = np.sqrt(dx**2 + dy**2)
            
            # Apply repulsive force if too close
            if distance < min_distance and distance > 0:
                force_magnitude = (min_distance - distance) / distance
                avoidance_force_x += force_magnitude * dx * 0.5
                avoidance_force_y += force_magnitude * dy * 0.5
        
        # Apply avoidance forces
        robot.x += avoidance_force_x * self.dt
        robot.y += avoidance_force_y * self.dt
    
    def process_vision(self):
        """Process vision system to detect leader"""
        if not self.vision_enabled:
            return
            
        # Create synthetic camera image
        all_robots = [self.leader] + self.followers
        camera_image = self.vision.create_synthetic_image(all_robots)
        
        # Detect leader robot
        detected_pos = self.vision.detect_leader(camera_image, target_color='red')
        
        if detected_pos is not None:
            self.detected_leader_pos = detected_pos
        
        # Store camera image for visualization update
        self.current_camera_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2RGB)
    
    def control_loop(self):
        """Main control loop"""
        while self.running:
            try:
                # Update leader motion
                self.update_leader_motion()
                
                # Process vision system
                self.process_vision()
                
                # Update followers based on detected leader position
                all_robots = [self.leader] + self.followers
                
                for i, follower in enumerate(self.followers):
                    # Get formation target
                    target_x, target_y = self.get_formation_target(i)
                    
                    # Update follower control
                    self.update_follower_control(follower, target_x, target_y)
                    
                    # Apply collision avoidance
                    self.collision_avoidance(follower, all_robots)
                
                time.sleep(self.dt)
                
            except Exception as e:
                print(f"Control loop error: {e}")
                break
    
    def update_visualization(self, frame):
        """Update visualization"""
        # Update main simulation view
        self.ax_main.clear()
        self.ax_main.set_xlim(-12, 4)
        self.ax_main.set_ylim(-8, 8)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        
        # Title with vision status
        vision_status = "🟢 VISION ON" if self.vision_enabled else "🔴 VISION OFF"
        self.ax_main.set_title(f'Vision-Based Leader-Follower System | {vision_status} | Formation: {self.current_formation.upper()}', 
                              fontsize=14, fontweight='bold')
                              
        # Update camera view
        self.ax_camera.clear()
        self.ax_camera.set_xlim(0, 640)
        self.ax_camera.set_ylim(0, 480)
        self.ax_camera.set_aspect('equal')
        self.ax_camera.set_title('Camera View (Robot Detection)', fontsize=12)
        self.ax_camera.invert_yaxis()
        
        # Display camera image if available
        if self.current_camera_image is not None:
            self.ax_camera.imshow(self.current_camera_image)
            
            # Show detection result
            if self.detected_leader_pos is not None:
                pixel_x, pixel_y = self.vision.world_to_camera(
                    self.detected_leader_pos[0], self.detected_leader_pos[1]
                )
                circle = plt.Circle((pixel_x, pixel_y), 20, color='yellow', fill=False, linewidth=3)
                self.ax_camera.add_patch(circle)
                self.ax_camera.text(10, 30, f'Leader Detected: ({self.detected_leader_pos[0]:.2f}, {self.detected_leader_pos[1]:.2f})', 
                                  color='lime', fontsize=10, fontweight='bold')
            else:
                self.ax_camera.text(10, 30, 'Leader Not Detected', color='red', fontsize=10, fontweight='bold')
        else:
            # Show empty camera view
            self.ax_camera.text(320, 240, 'Camera View', ha='center', va='center', fontsize=12, color='gray')
        
        # Update trails
        all_robots = [self.leader] + self.followers
        for robot in all_robots:
            trail = self.trails[robot.name]
            trail['x'].append(robot.x)
            trail['y'].append(robot.y)
            
            # Limit trail length
            if len(trail['x']) > 100:
                trail['x'].pop(0)
                trail['y'].pop(0)
            
            # Plot trail
            if len(trail['x']) > 1:
                self.ax_main.plot(trail['x'], trail['y'], color=robot.color, alpha=0.3, linewidth=1)
        
        # Plot robots
        colors = {'red': '🔴', 'blue': '🔵', 'green': '🟢', 'orange': '🟠'}
        
        # Leader
        self.ax_main.scatter(self.leader.x, self.leader.y, c=self.leader.color, s=200, 
                           marker=self.leader.marker, edgecolors='black', linewidth=2, 
                           label=f'{colors[self.leader.color]} Leader', zorder=5)
        
        # Detected leader position
        if self.detected_leader_pos is not None:
            self.ax_main.scatter(self.detected_leader_pos[0], self.detected_leader_pos[1], 
                               c='yellow', s=150, marker='x', linewidth=3,
                               label='📍 Detected Leader', zorder=6)
        
        # Followers and targets
        for i, follower in enumerate(self.followers):
            # Plot follower
            self.ax_main.scatter(follower.x, follower.y, c=follower.color, s=150, 
                               marker=follower.marker, edgecolors='black', linewidth=2,
                               label=f'{colors[follower.color]} {follower.name}', zorder=5)
            
            # Plot target position
            target_x, target_y = self.get_formation_target(i)
            self.ax_main.scatter(target_x, target_y, c=follower.color, s=100, 
                               marker='x', alpha=0.7, zorder=3)
        
        # Add legend and info
        self.ax_main.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Add control instructions
        info_text = (
            "🎮 CONTROLS:\n"
            "   V = Toggle Vision\n"
            "   SPACEBAR = Switch Formation\n"
            "   Ctrl+C = Stop\n"
            f"📊 STATUS:\n"
            f"   Formation: {self.current_formation.upper()}\n"
            f"   Vision: {'ON' if self.vision_enabled else 'OFF'}\n"
            f"   Detected: {'YES' if self.detected_leader_pos else 'NO'}"
        )
        self.ax_main.text(-11.5, -7, info_text, fontsize=9, 
                         bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
    
    def on_key_press(self, event):
        """Handle key press events"""
        if event.key == ' ':  # Spacebar - switch formation
            self.formation_index = (self.formation_index + 1) % len(self.formation_keys)
            self.current_formation = self.formation_keys[self.formation_index]
            print(f"🔄 Switched to {self.current_formation.upper()} formation")
        
        elif event.key.lower() == 'v':  # V - toggle vision
            self.vision_enabled = not self.vision_enabled
            vision_status = "ON" if self.vision_enabled else "OFF"
            print(f"🎥 Vision system: {vision_status}")
            if not self.vision_enabled:
                self.detected_leader_pos = None
    
    def run(self):
        """Run the demo"""
        print("🚀 Starting Vision-Based Multi-Robot Formation Demo...")
        print("🔴 Red star = Leader (actual)")
        print("📍 Yellow X = Leader (detected by vision)")
        print("🔵🟢🟠 Colored shapes = Followers")
        print("❌ X marks = Target formation positions")
        print("🎮 CONTROLS:")
        print("   V = Toggle vision system ON/OFF")
        print("   SPACEBAR = Switch formation patterns")
        print("   Ctrl+C = Stop demo")
        print("📋 Available formations: TRIANGLE → LINE → CIRCLE → V-SHAPE → (repeat)")
        
        # Connect key press handler
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Start control thread
        self.control_thread.start()
        
        # Start animation
        animation = FuncAnimation(self.fig, self.update_visualization, interval=50, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n🛑 Stopping vision-based demo...")
        finally:
            self.running = False

if __name__ == "__main__":
    try:
        controller = VisionBasedSwarmController()
        controller.run()
    except KeyboardInterrupt:
        print("\n🛑 Demo stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc() 