"""
Scene Generator for Advanced Simulation Environments

This module provides procedural generation of diverse simulation environments
for testing multi-robot leader-follower systems. Features include:
- Multiple environment types (office, warehouse, campus, urban, emergency)
- Procedural obstacle placement
- Dynamic element generation
- Progressive difficulty scaling
- Performance testing scenarios

Author: Modern Swarm Leader-Follower System
Phase: 6 - Advanced Simulation Environments
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, Circle, Polygon
import random
import math
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import time

class EnvironmentType(Enum):
    """Available environment types."""
    OFFICE = "office"
    WAREHOUSE = "warehouse"
    CAMPUS = "campus"
    URBAN = "urban"
    EMERGENCY = "emergency"
    OBSTACLE_COURSE = "obstacle_course"
    MAZE = "maze"
    OPEN_FIELD = "open_field"

class ObstacleType(Enum):
    """Types of obstacles."""
    STATIC_CIRCLE = "static_circle"
    STATIC_RECTANGLE = "static_rectangle"
    DYNAMIC_CIRCLE = "dynamic_circle"
    DYNAMIC_RECTANGLE = "dynamic_rectangle"
    WALL = "wall"
    FURNITURE = "furniture"
    VEHICLE = "vehicle"
    DEBRIS = "debris"

@dataclass
class Obstacle:
    """Obstacle definition."""
    obstacle_type: ObstacleType
    position: Tuple[float, float]
    size: Tuple[float, float]  # (width, height) or (radius, _)
    color: str = "red"
    velocity: Tuple[float, float] = (0.0, 0.0)
    behavior: Optional[str] = None
    active: bool = True

@dataclass
class Environment:
    """Complete environment definition."""
    name: str
    environment_type: EnvironmentType
    bounds: Tuple[float, float, float, float]  # (x_min, x_max, y_min, y_max)
    obstacles: List[Obstacle]
    start_position: Tuple[float, float]
    goal_position: Optional[Tuple[float, float]]
    metadata: Dict[str, Any]

class SceneGenerator:
    """
    Procedural scene generator for multi-robot simulation environments.
    
    Creates diverse, challenging environments for testing formation control,
    obstacle avoidance, and coordination algorithms.
    """
    
    def __init__(self, seed: Optional[int] = None):
        """
        Initialize scene generator.
        
        Args:
            seed: Random seed for reproducible generation
        """
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        self.environments = {}
        self.current_environment = None
        
        # Default parameters
        self.default_bounds = (-20, 20, -20, 20)
        self.robot_size = 0.5  # Robot radius for collision detection
        self.safety_margin = 0.2  # Additional margin around robots
        
        print("🎮 Scene Generator initialized!")
    
    def generate_environment(self, 
                           env_type: EnvironmentType,
                           difficulty: int = 1,
                           custom_params: Optional[Dict] = None) -> Environment:
        """
        Generate a complete environment based on type and difficulty.
        
        Args:
            env_type: Type of environment to generate
            difficulty: Difficulty level (1-5)
            custom_params: Custom parameters for generation
            
        Returns:
            Generated Environment object
        """
        print(f"🏗️ Generating {env_type.value} environment (difficulty {difficulty})...")
        
        params = custom_params or {}
        
        if env_type == EnvironmentType.OBSTACLE_COURSE:
            return self._generate_obstacle_course(difficulty, params)
        elif env_type == EnvironmentType.OFFICE:
            return self._generate_office_environment(difficulty, params)
        elif env_type == EnvironmentType.WAREHOUSE:
            return self._generate_warehouse_environment(difficulty, params)
        elif env_type == EnvironmentType.OPEN_FIELD:
            return self._generate_open_field(difficulty, params)
        else:
            # Default to obstacle course
            return self._generate_obstacle_course(difficulty, params)
    
    def _generate_obstacle_course(self, difficulty: int, params: Dict) -> Environment:
        """Generate obstacle course for formation testing."""
        bounds = params.get('bounds', self.default_bounds)
        obstacles = []
        
        # Create obstacle course with increasing difficulty
        obstacle_count = 5 + difficulty * 3
        min_spacing = max(4.0 - difficulty * 0.3, 2.0)
        
        # Slalom obstacles
        for i in range(obstacle_count):
            x_pos = bounds[0] + 5 + i * (min_spacing + 2)
            y_offset = 3 * math.sin(i * math.pi / 3)  # Sinusoidal pattern
            y_pos = y_offset
            
            # Alternate between circles and rectangles
            if i % 2 == 0:
                obstacle_size = random.uniform(0.8, 1.5)
                obstacles.append(
                    Obstacle(ObstacleType.STATIC_CIRCLE, (x_pos, y_pos),
                             (obstacle_size, obstacle_size), "red")
                )
            else:
                width = random.uniform(1.0, 2.0)
                height = random.uniform(1.0, 2.0)
                obstacles.append(
                    Obstacle(ObstacleType.STATIC_RECTANGLE, (x_pos, y_pos),
                             (width, height), "blue")
                )
        
        # Add dynamic obstacles for higher difficulty
        if difficulty >= 3:
            dynamic_count = difficulty - 2
            for _ in range(dynamic_count):
                dyn_x = random.uniform(bounds[0] + 5, bounds[1] - 5)
                dyn_y = random.uniform(bounds[2] + 3, bounds[3] - 3)
                velocity = (random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5))
                
                obstacles.append(
                    Obstacle(ObstacleType.DYNAMIC_CIRCLE, (dyn_x, dyn_y),
                             (1.2, 1.2), "purple", velocity, "random_walk")
                )
        
        return Environment(
            name=f"Obstacle Course (Difficulty {difficulty})",
            environment_type=EnvironmentType.OBSTACLE_COURSE,
            bounds=bounds,
            obstacles=obstacles,
            start_position=(bounds[0] + 2, 0),
            goal_position=(bounds[1] - 2, 0),
            metadata={'difficulty': difficulty, 'obstacle_count': obstacle_count, 'min_spacing': min_spacing}
        )
    
    def _generate_office_environment(self, difficulty: int, params: Dict) -> Environment:
        """Generate office building environment with corridors and rooms."""
        bounds = params.get('bounds', self.default_bounds)
        obstacles = []
        
        # Office walls and rooms
        room_count = min(2 + difficulty, 8)
        corridor_width = max(3.0 - difficulty * 0.3, 1.5)
        
        # Simple corridor with obstacles
        for i in range(room_count):
            x_pos = bounds[0] + 5 + i * 6
            y_pos = random.uniform(-3, 3)
            size = random.uniform(1.0, 2.0)
            
            obstacles.append(
                Obstacle(ObstacleType.STATIC_RECTANGLE, (x_pos, y_pos),
                         (size, size), "brown")
            )
        
        return Environment(
            name=f"Office Building (Difficulty {difficulty})",
            environment_type=EnvironmentType.OFFICE,
            bounds=bounds,
            obstacles=obstacles,
            start_position=(bounds[0] + 2, 0),
            goal_position=(bounds[1] - 2, 0),
            metadata={'difficulty': difficulty, 'corridor_width': corridor_width, 'room_count': room_count}
        )
    
    def _generate_warehouse_environment(self, difficulty: int, params: Dict) -> Environment:
        """Generate warehouse environment with shelving and cargo."""
        bounds = params.get('bounds', self.default_bounds)
        obstacles = []
        
        # Warehouse shelving
        shelf_count = 3 + difficulty * 2
        for i in range(shelf_count):
            x_pos = bounds[0] + 5 + i * 6
            y_pos = random.uniform(-5, 5)
            
            obstacles.append(
                Obstacle(ObstacleType.STATIC_RECTANGLE, (x_pos, y_pos),
                         (1.5, 4.0), "orange")
            )
        
        # Add dynamic forklifts
        if difficulty >= 2:
            forklift_count = difficulty - 1
            for _ in range(forklift_count):
                forklift_x = random.uniform(bounds[0] + 3, bounds[1] - 3)
                forklift_y = random.uniform(bounds[2] + 3, bounds[3] - 3)
                velocity = (random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3))
                
                obstacles.append(
                    Obstacle(ObstacleType.DYNAMIC_RECTANGLE, (forklift_x, forklift_y),
                             (2.0, 1.0), "purple", velocity, "patrol")
                )
        
        return Environment(
            name=f"Warehouse (Difficulty {difficulty})",
            environment_type=EnvironmentType.WAREHOUSE,
            bounds=bounds,
            obstacles=obstacles,
            start_position=(bounds[0] + 2, bounds[2] + 2),
            goal_position=(bounds[1] - 2, bounds[3] - 2),
            metadata={'difficulty': difficulty, 'shelf_count': shelf_count}
        )
    
    def _generate_open_field(self, difficulty: int, params: Dict) -> Environment:
        """Generate open field for formation testing."""
        bounds = params.get('bounds', self.default_bounds)
        obstacles = []
        
        # Minimal obstacles for formation precision testing
        if difficulty >= 2:
            sparse_obstacle_count = difficulty
            for _ in range(sparse_obstacle_count):
                obs_x = random.uniform(bounds[0] + 5, bounds[1] - 5)
                obs_y = random.uniform(bounds[2] + 5, bounds[3] - 5)
                obs_size = random.uniform(0.5, 1.0)
                
                obstacles.append(
                    Obstacle(ObstacleType.STATIC_CIRCLE, (obs_x, obs_y),
                             (obs_size, obs_size), "gray")
                )
        
        return Environment(
            name=f"Open Field (Difficulty {difficulty})",
            environment_type=EnvironmentType.OPEN_FIELD,
            bounds=bounds,
            obstacles=obstacles,
            start_position=(bounds[0] + 3, 0),
            goal_position=(bounds[1] - 3, 0),
            metadata={'difficulty': difficulty, 'obstacle_count': len(obstacles)}
        )
    
    def update_dynamic_obstacles(self, environment: Environment, dt: float):
        """Update positions of dynamic obstacles."""
        for obstacle in environment.obstacles:
            if obstacle.obstacle_type in [ObstacleType.DYNAMIC_CIRCLE, ObstacleType.DYNAMIC_RECTANGLE]:
                if obstacle.velocity != (0.0, 0.0):
                    # Update position
                    new_x = obstacle.position[0] + obstacle.velocity[0] * dt
                    new_y = obstacle.position[1] + obstacle.velocity[1] * dt
                    
                    # Boundary bouncing
                    if new_x <= environment.bounds[0] or new_x >= environment.bounds[1]:
                        obstacle.velocity = (-obstacle.velocity[0], obstacle.velocity[1])
                    if new_y <= environment.bounds[2] or new_y >= environment.bounds[3]:
                        obstacle.velocity = (obstacle.velocity[0], -obstacle.velocity[1])
                    
                    obstacle.position = (
                        max(environment.bounds[0], min(environment.bounds[1], new_x)),
                        max(environment.bounds[2], min(environment.bounds[3], new_y))
                    )
    
    def get_environment_info(self, environment: Environment) -> Dict:
        """Get comprehensive information about an environment."""
        static_count = len([obs for obs in environment.obstacles 
                           if obs.obstacle_type in [ObstacleType.STATIC_CIRCLE, ObstacleType.STATIC_RECTANGLE, 
                                                   ObstacleType.WALL, ObstacleType.FURNITURE]])
        dynamic_count = len([obs for obs in environment.obstacles 
                            if obs.obstacle_type in [ObstacleType.DYNAMIC_CIRCLE, ObstacleType.DYNAMIC_RECTANGLE]])
        
        return {
            'name': environment.name,
            'type': environment.environment_type.value,
            'bounds': environment.bounds,
            'total_obstacles': len(environment.obstacles),
            'static_obstacles': static_count,
            'dynamic_obstacles': dynamic_count,
            'start_position': environment.start_position,
            'goal_position': environment.goal_position,
            'metadata': environment.metadata
        }


# Test the scene generator
if __name__ == "__main__":
    print("🧪 Testing Scene Generator...")
    
    generator = SceneGenerator(seed=42)
    
    # Test different environment types
    environments = [
        (EnvironmentType.OBSTACLE_COURSE, 3),
        (EnvironmentType.OFFICE, 2),
        (EnvironmentType.WAREHOUSE, 4),
        (EnvironmentType.OPEN_FIELD, 1)
    ]
    
    for env_type, difficulty in environments:
        env = generator.generate_environment(env_type, difficulty)
        info = generator.get_environment_info(env)
        
        print(f"\n✅ Generated: {info['name']}")
        print(f"   Total obstacles: {info['total_obstacles']}")
        print(f"   Static: {info['static_obstacles']}, Dynamic: {info['dynamic_obstacles']}")
        print(f"   Bounds: {info['bounds']}")
    
    print("\n🎉 Scene Generator tests completed!") 