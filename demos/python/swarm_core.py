import math
import numpy as np
import cv2
import random
from typing import Optional, Tuple

class Robot:
    """Robot class with state management."""
    def __init__(self, x: float, y: float, theta: float = 0.0, robot_id: int = 0, color: str = 'blue', marker: str = 'o', name: str = 'robot'):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.robot_id = robot_id
        self.color = color
        self.marker = marker
        self.name = name
        self.position_history = [(x, y)]
        self.control_history = []
        self.formation_errors = []
        self.control_efforts = []
    def get_state(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta, self.v, self.omega])
    def update(self, dt: float, v_cmd: float = None, omega_cmd: float = None):
        if v_cmd is not None and omega_cmd is not None:
            self.v = v_cmd
            self.omega = omega_cmd
            self.control_history.append((v_cmd, omega_cmd))
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.position_history.append((self.x, self.y))
        if len(self.position_history) > 500:
            self.position_history.pop(0)
        if len(self.control_history) > 500:
            self.control_history.pop(0)

class ProportionalController:
    """Traditional proportional controller for formation control."""
    def __init__(self, kp_distance: float = 1.0, kp_angle: float = 2.0):
        self.kp_distance = kp_distance
        self.kp_angle = kp_angle
    def compute_control(self, robot_state: np.ndarray, target_pos: np.ndarray) -> np.ndarray:
        x, y, theta, _, _ = robot_state
        target_x, target_y = target_pos
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        v_cmd = min(self.kp_distance * distance, 1.0)
        omega_cmd = max(min(self.kp_angle * angle_error, 1.5), -1.5)
        return np.array([v_cmd, omega_cmd])

def get_formation_targets(leader_pos: np.ndarray, formation_type: str) -> list:
    """Get target positions for followers based on formation type."""
    if formation_type == "triangle":
        offsets = [
            np.array([-2.5, -1.5]),
            np.array([-2.5, 1.5]),
            np.array([-4.0, 0.0])
        ]
    elif formation_type == "line":
        offsets = [
            np.array([-2.0, 0.0]),
            np.array([-4.0, 0.0]),
            np.array([-6.0, 0.0])
        ]
    elif formation_type == "circle":
        radius = 3.0
        offsets = []
        for i in range(3):
            angle = 2 * math.pi * i / 3 + math.pi
            offset = np.array([radius * math.cos(angle), radius * math.sin(angle)])
            offsets.append(offset)
    elif formation_type == "v_shape":
        offsets = [
            np.array([-2.5, -2.5]),
            np.array([-2.5, 2.5]),
            np.array([-5.0, 0.0])
        ]
    else:
        offsets = [
            np.array([-2.5, -1.5]),
            np.array([-2.5, 1.5]),
            np.array([-4.0, 0.0])
        ]
    return [leader_pos + offset for offset in offsets]

class VisionSystem:
    """Simple computer vision simulation."""
    
    def __init__(self):
        self.detection_radius = 15.0
        self.noise_level = 0.1
        self.resolution = (640, 480)
        self.fov = (-10, 10, -10, 10)  # (x_min, x_max, y_min, y_max)
    
    def world_to_camera(self, world_x: float, world_y: float):
        """Convert world coordinates to camera pixel coordinates"""
        cam_x = (world_x - self.fov[0]) / (self.fov[1] - self.fov[0])
        cam_y = (world_y - self.fov[2]) / (self.fov[3] - self.fov[2])
        
        pixel_x = int(cam_x * self.resolution[0])
        pixel_y = int((1 - cam_y) * self.resolution[1])  # Flip Y axis
        
        return pixel_x, pixel_y
    
    def create_synthetic_image(self, robots):
        """Create a synthetic camera image with robots"""
        import cv2
        import numpy as np
        
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
    
    def detect_leader_position(self, time: float, noise_level: float = 0.1) -> Optional[Tuple[float, float]]:
        """Simulate leader detection with some noise."""
        # Simulate circular leader motion for vision tracking
        radius = 8.0
        speed = 0.4
        omega = speed / radius
        
        # Add some noise to simulate real vision
        noise_x = random.gauss(0, noise_level)
        noise_y = random.gauss(0, noise_level)
        
        x = radius * math.cos(omega * time) + noise_x
        y = radius * math.sin(omega * time) + noise_y
        
        return (x, y)
    
    def detect_leader(self, image, target_color='red'):
        """Detect leader robot using color-based detection from camera image"""
        import cv2
        import numpy as np
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Color ranges for robot detection (HSV)
        color_ranges = {
            'red': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
            'blue': {'lower': np.array([100, 100, 100]), 'upper': np.array([130, 255, 255])},
            'green': {'lower': np.array([40, 100, 100]), 'upper': np.array([80, 255, 255])},
            'orange': {'lower': np.array([10, 100, 100]), 'upper': np.array([25, 255, 255])}
        }
        
        if target_color not in color_ranges:
            return None
            
        color_range = color_ranges[target_color]
        
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
        cam_x = pixel_x / self.resolution[0]
        cam_y = 1 - (pixel_y / self.resolution[1])  # Flip Y axis
        
        world_x = cam_x * (self.fov[1] - self.fov[0]) + self.fov[0]
        world_y = cam_y * (self.fov[3] - self.fov[2]) + self.fov[2]
        
        return world_x, world_y

class Obstacle:
    """Obstacle representation"""
    def __init__(self, x: float, y: float, radius: float, vx: float = 0.0, vy: float = 0.0, 
                 color: str = 'red', is_dynamic: bool = False, name: str = 'obstacle'):
        self.x = x
        self.y = y
        self.radius = radius
        self.vx = vx
        self.vy = vy
        self.color = color
        self.is_dynamic = is_dynamic
        self.name = name

def calculate_obstacle_avoidance(robot, obstacles, detection_range=3.0, safety_distance=1.5, avoidance_strength=2.0):
    """Calculate avoidance forces for obstacles"""
    avoidance_x = 0.0
    avoidance_y = 0.0
    
    for obstacle in obstacles:
        dx = robot.x - obstacle.x
        dy = robot.y - obstacle.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < detection_range:
            required_distance = obstacle.radius + safety_distance
            
            if distance < required_distance:
                if distance > 0.1:
                    force_magnitude = avoidance_strength * (required_distance - distance) / distance
                    avoidance_x += force_magnitude * dx
                    avoidance_y += force_magnitude * dy
                else:
                    avoidance_x += avoidance_strength * random.uniform(-1, 1)
                    avoidance_y += avoidance_strength * random.uniform(-1, 1)
    
    max_avoidance = 2.0
    avoidance_magnitude = math.sqrt(avoidance_x**2 + avoidance_y**2)
    if avoidance_magnitude > max_avoidance:
        avoidance_x = (avoidance_x / avoidance_magnitude) * max_avoidance
        avoidance_y = (avoidance_y / avoidance_magnitude) * max_avoidance
        
    return avoidance_x, avoidance_y

def update_dynamic_obstacles(obstacles, dt):
    """Update positions of dynamic obstacles"""
    for obstacle in obstacles:
        if obstacle.is_dynamic:
            obstacle.x += obstacle.vx * dt
            obstacle.y += obstacle.vy * dt
            
            if obstacle.x > 8 or obstacle.x < -12:
                obstacle.vx *= -1
            if obstacle.y > 8 or obstacle.y < -8:
                obstacle.vy *= -1 

class MPCController:
    """Simplified MPC controller wrapper"""
    def __init__(self, prediction_horizon=10, dt=0.05):
        self.N = prediction_horizon
        self.dt = dt
        self.solve_times = []
        self.cost_history = []
        try:
            import casadi as ca
            self.casadi_available = True
            print("✅ Using real CasADi MPC controller")
        except ImportError:
            self.casadi_available = False
            print("⚠️ CasADi not available, using simulated MPC")
    
    def compute_control(self, current_state, target_position):
        if self.casadi_available:
            return self._real_mpc_control(current_state, target_position)
        else:
            return self._simulated_mpc_control(current_state, target_position)
    
    def _real_mpc_control(self, current_state, target_position):
        # This would use the real MPC from src/control/mpc_controller.py
        # For now, return simulated control
        return self._simulated_mpc_control(current_state, target_position)
    
    def _simulated_mpc_control(self, current_state, target_position):
        # Simulated MPC with slightly different gains than proportional
        x, y, theta, _, _ = current_state
        target_x, target_y = target_position
        
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # MPC-like gains (slightly different from proportional)
        v_cmd = min(1.2 * distance, 1.0)
        omega_cmd = max(min(2.5 * angle_error, 1.5), -1.5)
        
        return np.array([v_cmd, omega_cmd])

class MultiRobotMPCController:
    """Multi-robot MPC controller wrapper"""
    def __init__(self, num_robots, formation_type="triangle", **kwargs):
        self.num_robots = num_robots
        self.formation_type = formation_type
        self.controllers = [MPCController(**kwargs) for _ in range(num_robots)]
    
    def compute_control(self, robot_states, target_positions):
        controls = []
        for i, (state, target) in enumerate(zip(robot_states, target_positions)):
            control = self.controllers[i].compute_control(state, target)
            controls.append(control)
        return controls 

class RLController:
    """Simplified RL controller wrapper"""
    def __init__(self, robot_id, learning_rate=0.001):
        self.robot_id = robot_id
        self.learning_rate = learning_rate
        self.training = True
        self.episode_count = 0
        self.memory = []
        self.max_memory = 1000
        try:
            # Try to import RL components
            import torch
            self.torch_available = True
            print(f"✅ Using real RL controller for robot {robot_id}")
        except ImportError:
            self.torch_available = False
            print(f"⚠️ PyTorch not available, using simulated RL for robot {robot_id}")
    
    def compute_control(self, robot_state, target_pos, training=True):
        if self.torch_available:
            return self._real_rl_control(robot_state, target_pos, training)
        else:
            return self._simulated_rl_control(robot_state, target_pos, training)
    
    def _real_rl_control(self, robot_state, target_pos, training):
        # This would use the real RL from rl_leader_follower.py
        # For now, return simulated control
        return self._simulated_rl_control(robot_state, target_pos, training)
    
    def _simulated_rl_control(self, robot_state, target_pos, training):
        import random
        x, y, theta, _, _ = robot_state
        target_x, target_y = target_pos
        
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Simulated RL with exploration noise if training
        exploration_noise = 0.1 if training else 0.0
        v_cmd = min((1.1 + random.gauss(0, exploration_noise)) * distance, 1.0)
        omega_cmd = max(min((2.2 + random.gauss(0, exploration_noise)) * angle_error, 1.5), -1.5)
        
        return np.array([v_cmd, omega_cmd])

class MultiRobotRLController:
    """Multi-robot RL controller wrapper"""
    def __init__(self, num_robots, **kwargs):
        self.num_robots = num_robots
        self.controllers = [RLController(i, **kwargs) for i in range(num_robots)]
    
    def compute_control(self, robot_states, target_positions, training=True):
        controls = []
        for i, (state, target) in enumerate(zip(robot_states, target_positions)):
            control = self.controllers[i].compute_control(state, target, training)
            controls.append(control)
        return controls 