"""
Reinforcement Learning Multi-Robot Leader-Follower System

This demo implements Deep Q-Network (DQN) based formation control as an alternative
to traditional control methods. Features:
- RL-based formation control vs. traditional proportional control
- Real-time learning and adaptation
- Formation switching with learned policies
- Performance comparison with other methods

Controls:
- SPACEBAR: Switch formation patterns  
- R: Toggle between RL and Proportional control
- T: Start/stop training mode
- S: Save trained model
- L: Load trained model
- Ctrl+C: Stop demo

Author: Modern Swarm Leader-Follower System  
Phase: 5 - Advanced Control (RL)
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import math
from typing import List, Dict, Tuple, Optional
import sys
import os
import pickle
from collections import deque
import random

# Simple neural network implementation (avoiding heavy dependencies)
class SimpleNeuralNetwork:
    """Simple feedforward neural network for DQN."""
    
    def __init__(self, input_size: int, hidden_size: int, output_size: int, learning_rate: float = 0.001):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.lr = learning_rate
        
        # Initialize weights with small random values
        self.W1 = np.random.randn(input_size, hidden_size) * 0.1
        self.b1 = np.zeros((1, hidden_size))
        self.W2 = np.random.randn(hidden_size, output_size) * 0.1
        self.b2 = np.zeros((1, output_size))
        
        # For momentum
        self.vW1 = np.zeros_like(self.W1)
        self.vb1 = np.zeros_like(self.b1)
        self.vW2 = np.zeros_like(self.W2)
        self.vb2 = np.zeros_like(self.b2)
        self.beta = 0.9
    
    def relu(self, x):
        return np.maximum(0, x)
    
    def relu_derivative(self, x):
        return (x > 0).astype(float)
    
    def forward(self, x):
        self.z1 = np.dot(x, self.W1) + self.b1
        self.a1 = self.relu(self.z1)
        self.z2 = np.dot(self.a1, self.W2) + self.b2
        return self.z2
    
    def backward(self, x, y, output):
        m = x.shape[0]
        
        # Backpropagation
        dz2 = output - y
        dW2 = (1/m) * np.dot(self.a1.T, dz2)
        db2 = (1/m) * np.sum(dz2, axis=0, keepdims=True)
        
        da1 = np.dot(dz2, self.W2.T)
        dz1 = da1 * self.relu_derivative(self.z1)
        dW1 = (1/m) * np.dot(x.T, dz1)
        db1 = (1/m) * np.sum(dz1, axis=0, keepdims=True)
        
        # Update with momentum
        self.vW2 = self.beta * self.vW2 + (1 - self.beta) * dW2
        self.vb2 = self.beta * self.vb2 + (1 - self.beta) * db2
        self.vW1 = self.beta * self.vW1 + (1 - self.beta) * dW1
        self.vb1 = self.beta * self.vb1 + (1 - self.beta) * db1
        
        self.W2 -= self.lr * self.vW2
        self.b2 -= self.lr * self.vb2
        self.W1 -= self.lr * self.vW1
        self.b1 -= self.lr * self.vb1

class DQNAgent:
    """Deep Q-Network agent for formation control."""
    
    def __init__(self, state_size: int, action_size: int, learning_rate: float = 0.001):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = learning_rate
        
        # Neural networks (main and target)
        self.q_network = SimpleNeuralNetwork(state_size, 64, action_size, learning_rate)
        self.target_network = SimpleNeuralNetwork(state_size, 64, action_size, learning_rate)
        self.update_target_network()
        
        # Training parameters
        self.batch_size = 32
        self.gamma = 0.95  # discount factor
        self.update_target_freq = 100
        self.train_step = 0
        
        # Performance tracking
        self.losses = []
        self.rewards = []
        self.episode_rewards = []
    
    def update_target_network(self):
        """Copy weights from main network to target network."""
        self.target_network.W1 = self.q_network.W1.copy()
        self.target_network.b1 = self.q_network.b1.copy()
        self.target_network.W2 = self.q_network.W2.copy()
        self.target_network.b2 = self.q_network.b2.copy()
    
    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay buffer."""
        self.memory.append((state, action, reward, next_state, done))
    
    def act(self, state, training=True):
        """Choose action using epsilon-greedy policy."""
        if training and np.random.random() <= self.epsilon:
            return np.random.choice(self.action_size)
        
        q_values = self.q_network.forward(state.reshape(1, -1))
        return np.argmax(q_values[0])
    
    def replay(self):
        """Train the neural network on a batch of experiences."""
        if len(self.memory) < self.batch_size:
            return
        
        batch = random.sample(self.memory, self.batch_size)
        states = np.array([e[0] for e in batch])
        actions = np.array([e[1] for e in batch])
        rewards = np.array([e[2] for e in batch])
        next_states = np.array([e[3] for e in batch])
        dones = np.array([e[4] for e in batch])
        
        # Compute target Q-values
        current_q_values = self.q_network.forward(states)
        next_q_values = self.target_network.forward(next_states)
        
        target_q_values = current_q_values.copy()
        
        for i in range(self.batch_size):
            if dones[i]:
                target_q_values[i][actions[i]] = rewards[i]
            else:
                target_q_values[i][actions[i]] = rewards[i] + self.gamma * np.max(next_q_values[i])
        
        # Train the network
        self.q_network.backward(states, target_q_values, current_q_values)
        
        # Update target network periodically
        self.train_step += 1
        if self.train_step % self.update_target_freq == 0:
            self.update_target_network()
        
        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    
    def save_model(self, filepath: str):
        """Save the trained model."""
        model_data = {
            'W1': self.q_network.W1,
            'b1': self.q_network.b1,
            'W2': self.q_network.W2,
            'b2': self.q_network.b2,
            'epsilon': self.epsilon
        }
        with open(filepath, 'wb') as f:
            pickle.dump(model_data, f)
        print(f"🤖 Model saved to {filepath}")
    
    def load_model(self, filepath: str):
        """Load a trained model."""
        try:
            with open(filepath, 'rb') as f:
                model_data = pickle.load(f)
            
            self.q_network.W1 = model_data['W1']
            self.q_network.b1 = model_data['b1']
            self.q_network.W2 = model_data['W2']
            self.q_network.b2 = model_data['b2']
            self.epsilon = model_data['epsilon']
            self.update_target_network()
            print(f"🤖 Model loaded from {filepath}")
            return True
        except FileNotFoundError:
            print(f"❌ Model file {filepath} not found")
            return False

class Robot:
    """Robot class with state management."""
    def __init__(self, x: float, y: float, theta: float = 0.0, robot_id: int = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.robot_id = robot_id
        
        # Control history for analysis
        self.position_history = [(x, y)]
        self.control_history = []
        
        # Performance metrics
        self.formation_errors = []
        self.control_efforts = []
        self.rewards = []
    
    def get_state(self) -> np.ndarray:
        """Get robot state as numpy array."""
        return np.array([self.x, self.y, self.theta, self.v, self.omega])
    
    def update(self, dt: float, v_cmd: float = None, omega_cmd: float = None):
        """Update robot state with control inputs."""
        if v_cmd is not None and omega_cmd is not None:
            self.v = v_cmd
            self.omega = omega_cmd
            self.control_history.append((v_cmd, omega_cmd))
        
        # Update position using bicycle model
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Store position history
        self.position_history.append((self.x, self.y))
        
        # Limit history length for performance
        if len(self.position_history) > 500:
            self.position_history.pop(0)
        if len(self.control_history) > 500:
            self.control_history.pop(0)

class ProportionalController:
    """Traditional proportional controller for comparison."""
    
    def __init__(self, kp_distance: float = 1.0, kp_angle: float = 2.0):
        self.kp_distance = kp_distance
        self.kp_angle = kp_angle
    
    def compute_control(self, robot_state: np.ndarray, target_pos: np.ndarray) -> np.ndarray:
        """Compute proportional control."""
        x, y, theta, _, _ = robot_state
        target_x, target_y = target_pos
        
        # Distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Angle error
        angle_error = target_angle - theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Control commands
        v_cmd = min(self.kp_distance * distance, 1.0)  # Max speed limit
        omega_cmd = max(min(self.kp_angle * angle_error, 1.5), -1.5)  # Max angular speed
        
        return np.array([v_cmd, omega_cmd])

class RLFormationController:
    """RL-based formation controller."""
    
    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        
        # State: [robot_x, robot_y, robot_theta, target_x, target_y, distance_to_target, angle_to_target]
        self.state_size = 7
        
        # Actions: [stop, forward_slow, forward_fast, turn_left, turn_right, forward_left, forward_right]
        self.action_size = 7
        self.actions = [
            [0.0, 0.0],      # stop
            [0.3, 0.0],      # forward slow
            [0.8, 0.0],      # forward fast
            [0.0, 1.0],      # turn left
            [0.0, -1.0],     # turn right
            [0.4, 0.5],      # forward + left
            [0.4, -0.5],     # forward + right
        ]
        
        self.agent = DQNAgent(self.state_size, self.action_size)
        self.last_state = None
        self.last_action = None
        self.last_distance = None
    
    def get_state_representation(self, robot_state: np.ndarray, target_pos: np.ndarray) -> np.ndarray:
        """Convert robot state and target to RL state representation."""
        x, y, theta, v, omega = robot_state
        target_x, target_y = target_pos
        
        # Calculate distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx) - theta
        angle_to_target = math.atan2(math.sin(angle_to_target), math.cos(angle_to_target))
        
        return np.array([x, y, theta, target_x, target_y, distance, angle_to_target])
    
    def compute_reward(self, robot_state: np.ndarray, target_pos: np.ndarray, action: int) -> float:
        """Compute reward for RL training."""
        x, y, theta, v, omega = robot_state
        target_x, target_y = target_pos
        
        # Distance to target
        distance = math.sqrt((target_x - x)**2 + (target_y - y)**2)
        
        # Reward based on distance improvement
        reward = 0.0
        if self.last_distance is not None:
            distance_improvement = self.last_distance - distance
            reward += distance_improvement * 10.0  # Reward for getting closer
        
        # Penalty for being far from target
        reward -= distance * 0.1
        
        # Reward for being close to target
        if distance < 1.0:
            reward += 5.0
        if distance < 0.5:
            reward += 10.0
        
        # Small penalty for control effort
        control = self.actions[action]
        control_effort = abs(control[0]) + abs(control[1])
        reward -= control_effort * 0.01
        
        self.last_distance = distance
        return reward
    
    def compute_control(self, robot_state: np.ndarray, target_pos: np.ndarray, training: bool = True) -> np.ndarray:
        """Compute control using RL agent."""
        current_state = self.get_state_representation(robot_state, target_pos)
        
        # Choose action
        action = self.agent.act(current_state, training)
        
        # Get control commands
        control_commands = np.array(self.actions[action])
        
        # Training: store experience and learn
        if training and self.last_state is not None:
            reward = self.compute_reward(robot_state, target_pos, action)
            self.agent.remember(self.last_state, self.last_action, reward, current_state, False)
            self.agent.replay()
        
        # Update for next step
        self.last_state = current_state
        self.last_action = action
        
        return control_commands

class RLLeaderFollowerSystem:
    """RL-based multi-robot leader-follower system with performance comparison."""
    
    def __init__(self):
        # Initialize robots (1 leader + 3 followers)
        self.leader = Robot(0, 0, 0, robot_id=0)
        self.followers = [
            Robot(-2, -1.5, 0, robot_id=1),
            Robot(-2, 1.5, 0, robot_id=2), 
            Robot(-3, 0, 0, robot_id=3)
        ]
        
        # Controllers
        self.rl_controllers = [RLFormationController(i+1) for i in range(3)]
        self.prop_controller = ProportionalController(kp_distance=1.0, kp_angle=2.0)
        
        # Formation patterns
        self.formations = ["triangle", "line", "circle", "v_shape"]
        self.current_formation_idx = 0
        
        # Control mode
        self.use_rl = True
        self.training_mode = True
        self.last_formation_switch = time.time()
        
        # Performance tracking
        self.rl_metrics = {
            'rewards': [],
            'formation_errors': [],
            'control_efforts': [],
            'training_losses': []
        }
        self.prop_metrics = {
            'formation_errors': [],
            'control_efforts': []
        }
        
        # Simulation parameters
        self.dt = 0.05
        self.time = 0.0
        
        # Leader trajectory parameters
        self.leader_speed = 0.3
        self.leader_radius = 8.0
        
        # Episode tracking for RL
        self.episode = 0
        self.episode_step = 0
        self.episode_length = 1000  # steps per episode
        
        print(f"🤖 RL Leader-Follower System Initialized!")
        print(f"🎮 Controls:")
        print(f"   SPACEBAR: Switch formations")
        print(f"   R: Toggle RL/Proportional control")
        print(f"   T: Toggle training mode")
        print(f"   S: Save trained model")
        print(f"   L: Load trained model")
    
    def get_formation_targets(self, leader_pos: np.ndarray) -> List[np.ndarray]:
        """Get formation target positions."""
        formation_type = self.formations[self.current_formation_idx]
        
        if formation_type == "triangle":
            offsets = [
                np.array([-2.0, -1.5]),  # Left rear
                np.array([-2.0, 1.5]),   # Right rear
                np.array([-3.0, 0.0])    # Center rear
            ]
        elif formation_type == "line":
            offsets = [
                np.array([-2.0, 0.0]),
                np.array([-4.0, 0.0]),
                np.array([-6.0, 0.0])
            ]
        elif formation_type == "circle":
            radius = 2.5
            offsets = []
            for i in range(3):
                angle = 2 * math.pi * i / 3 + math.pi
                offset = np.array([radius * math.cos(angle), radius * math.sin(angle)])
                offsets.append(offset)
        else:  # v_shape
            offsets = [
                np.array([-2.0, -2.0]),  # Left wing
                np.array([-2.0, 2.0]),   # Right wing
                np.array([-4.0, 0.0])    # Center rear
            ]
        
        targets = []
        for offset in offsets:
            target = leader_pos + offset
            targets.append(target)
        
        return targets
    
    def update_leader(self):
        """Update leader with circular motion."""
        # Simple circular motion
        omega_leader = self.leader_speed / self.leader_radius
        self.leader.update(self.dt, self.leader_speed, omega_leader)
    
    def update_followers(self):
        """Update followers with RL or proportional control."""
        leader_pos = np.array([self.leader.x, self.leader.y])
        formation_targets = self.get_formation_targets(leader_pos)
        
        for i, follower in enumerate(self.followers):
            target_pos = formation_targets[i]
            
            if self.use_rl:
                # RL control
                control = self.rl_controllers[i].compute_control(
                    follower.get_state(), 
                    target_pos,
                    training=self.training_mode
                )
                
                # Store metrics
                if self.training_mode:
                    agent = self.rl_controllers[i].agent
                    if len(agent.memory) > 0:
                        self.rl_metrics['rewards'].append(agent.rewards[-1] if agent.rewards else 0)
            else:
                # Proportional control
                control = self.prop_controller.compute_control(follower.get_state(), target_pos)
            
            # Apply control with limits
            v_cmd = max(min(control[0], 1.0), -1.0)
            omega_cmd = max(min(control[1], 1.5), -1.5)
            
            follower.update(self.dt, v_cmd, omega_cmd)
            
            # Collision avoidance between followers
            for j, other_follower in enumerate(self.followers):
                if i != j:
                    distance = math.sqrt((follower.x - other_follower.x)**2 + 
                                       (follower.y - other_follower.y)**2)
                    if distance < 1.0:  # Minimum distance
                        # Simple repulsion
                        repulsion_x = follower.x - other_follower.x
                        repulsion_y = follower.y - other_follower.y
                        if distance > 0:
                            repulsion_x /= distance
                            repulsion_y /= distance
                        
                        follower.x += repulsion_x * 0.1
                        follower.y += repulsion_y * 0.1
    
    def switch_formation(self):
        """Switch to next formation pattern."""
        if time.time() - self.last_formation_switch > 1.0:  # Prevent rapid switching
            self.current_formation_idx = (self.current_formation_idx + 1) % len(self.formations)
            formation_name = self.formations[self.current_formation_idx]
            print(f"🔄 Formation switched to: {formation_name}")
            self.last_formation_switch = time.time()
    
    def toggle_controller(self):
        """Toggle between RL and proportional control."""
        self.use_rl = not self.use_rl
        controller_name = "RL" if self.use_rl else "Proportional"
        print(f"🎛️ Controller switched to: {controller_name}")
    
    def toggle_training(self):
        """Toggle training mode for RL."""
        self.training_mode = not self.training_mode
        mode = "ON" if self.training_mode else "OFF"
        print(f"🎓 Training mode: {mode}")
    
    def save_models(self):
        """Save all RL models."""
        try:
            os.makedirs('models', exist_ok=True)
            saved_count = 0
            for i, controller in enumerate(self.rl_controllers):
                filepath = f"models/rl_follower_{i+1}.pkl"
                try:
                    controller.agent.save_model(filepath)
                    print(f"🤖 Model saved to {filepath}")
                    saved_count += 1
                except Exception as e:
                    print(f"❌ Failed to save model {i+1}: {e}")
            
            if saved_count == len(self.rl_controllers):
                print(f"✅ All {saved_count} models saved successfully!")
            else:
                print(f"⚠️ Only {saved_count}/{len(self.rl_controllers)} models saved")
                
        except Exception as e:
            print(f"❌ Error creating models directory: {e}")
            raise
    
    def load_models(self):
        """Load all RL models."""
        success_count = 0
        for i, controller in enumerate(self.rl_controllers):
            filepath = f"models/rl_follower_{i+1}.pkl"
            if controller.agent.load_model(filepath):
                success_count += 1
        
        if success_count > 0:
            print(f"✅ Loaded {success_count} models successfully")
            self.training_mode = False  # Switch to inference mode
        else:
            print("❌ No models loaded - staying in training mode")
    
    def show_performance_metrics(self):
        """Display performance metrics."""
        print(f"\n📊 Performance Metrics:")
        print(f"Episode: {self.episode}, Step: {self.episode_step}")
        print(f"Controller: {'RL' if self.use_rl else 'Proportional'}")
        print(f"Training: {'ON' if self.training_mode else 'OFF'}")
        
        if self.use_rl and len(self.rl_metrics['rewards']) > 0:
            avg_reward = np.mean(self.rl_metrics['rewards'][-100:])
            print(f"Average Reward (last 100): {avg_reward:.3f}")
            
            for i, controller in enumerate(self.rl_controllers):
                agent = controller.agent
                print(f"Follower {i+1} - Epsilon: {agent.epsilon:.3f}, Memory: {len(agent.memory)}")
    
    def update(self):
        """Update simulation."""
        self.update_leader()
        self.update_followers()
        self.time += self.dt
        self.episode_step += 1
        
        # Handle episode transitions for RL
        if self.episode_step >= self.episode_length:
            self.episode += 1
            self.episode_step = 0
            if self.training_mode:
                print(f"🎯 Episode {self.episode} completed")
    
    def run_demo(self):
        """Run the RL leader-follower demo."""
        print(f"🚀 Starting RL Leader-Follower Demo...")
        print(f"Formation: {self.formations[self.current_formation_idx]}")
        print(f"Controller: {'RL' if self.use_rl else 'Proportional'}")
        
        # Set up the plot
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Main simulation plot
        ax1.set_xlim(-15, 15)
        ax1.set_ylim(-15, 15)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_title('RL Multi-Robot Leader-Follower System', fontsize=14, fontweight='bold')
        
        # Performance plot
        ax2.set_title('Training Performance', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        
        def on_key_press(event):
            if event.key == ' ':
                self.switch_formation()
            elif event.key == 'r':
                self.toggle_controller()
            elif event.key == 't':
                self.toggle_training()
            elif event.key == 's':
                self.save_models()
            elif event.key == 'l':
                self.load_models()
            elif event.key == 'p':
                self.show_performance_metrics()
        
        fig.canvas.mpl_connect('key_press_event', on_key_press)
        
        start_time = time.time()
        frame_count = 0
        running = True
        
        def cleanup_and_exit():
            nonlocal running
            running = False
            
        try:
            while running:
                # Update simulation
                self.update()
                
                # Update visualization every few frames for performance
                if frame_count % 5 == 0:
                    # Clear previous plots
                    ax1.clear()
                    ax2.clear()
                    
                    # Main simulation plot
                    ax1.set_xlim(-15, 15)
                    ax1.set_ylim(-15, 15)
                    ax1.set_aspect('equal')
                    ax1.grid(True, alpha=0.3)
                    
                    # Plot leader
                    leader_circle = patches.Circle((self.leader.x, self.leader.y), 0.5, 
                                                 color='red', alpha=0.8, label='Leader')
                    ax1.add_patch(leader_circle)
                    
                    # Plot leader trail
                    if len(self.leader.position_history) > 1:
                        leader_x = [pos[0] for pos in self.leader.position_history[-100:]]
                        leader_y = [pos[1] for pos in self.leader.position_history[-100:]]
                        ax1.plot(leader_x, leader_y, 'r-', alpha=0.3, linewidth=1)
                    
                    # Plot followers with different colors
                    colors = ['blue', 'green', 'orange']
                    labels = ['Follower 1', 'Follower 2', 'Follower 3']
                    
                    for i, (follower, color, label) in enumerate(zip(self.followers, colors, labels)):
                        # Robot circle
                        follower_circle = patches.Circle((follower.x, follower.y), 0.4, 
                                                       color=color, alpha=0.8, label=label)
                        ax1.add_patch(follower_circle)
                        
                        # Robot trail
                        if len(follower.position_history) > 1:
                            follower_x = [pos[0] for pos in follower.position_history[-50:]]
                            follower_y = [pos[1] for pos in follower.position_history[-50:]]
                            ax1.plot(follower_x, follower_y, color=color, alpha=0.3, linewidth=1)
                        
                        # Formation target
                        leader_pos = np.array([self.leader.x, self.leader.y])
                        targets = self.get_formation_targets(leader_pos)
                        target_x, target_y = targets[i]
                        ax1.plot(target_x, target_y, 'x', color=color, markersize=8, alpha=0.6)
                        ax1.plot([follower.x, target_x], [follower.y, target_y], 
                               '--', color=color, alpha=0.4)
                    
                    # Add information text
                    formation_name = self.formations[self.current_formation_idx]
                    controller_name = "RL" if self.use_rl else "Proportional"
                    training_status = "Training" if self.training_mode else "Inference"
                    
                    info_text = f"Formation: {formation_name}\nController: {controller_name}"
                    if self.use_rl:
                        info_text += f"\nMode: {training_status}"
                        info_text += f"\nEpisode: {self.episode}"
                    
                    ax1.text(0.02, 0.98, info_text, transform=ax1.transAxes, 
                           fontsize=10, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
                    
                    # Control instructions
                    controls_text = ("SPACEBAR: Switch formation | R: Toggle control | "
                                   "T: Toggle training | S: Save | L: Load | P: Metrics")
                    ax1.text(0.5, 0.02, controls_text, transform=ax1.transAxes, 
                           fontsize=9, ha='center',
                           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
                    
                    ax1.set_title('RL Multi-Robot Leader-Follower System', fontsize=14, fontweight='bold')
                    ax1.legend(loc='upper right')
                    
                    # Performance plot
                    ax2.set_title('Training Performance', fontsize=14, fontweight='bold')
                    ax2.grid(True, alpha=0.3)
                    
                    if self.use_rl and len(self.rl_metrics['rewards']) > 10:
                        # Moving average of rewards
                        window_size = min(50, len(self.rl_metrics['rewards']))
                        rewards = self.rl_metrics['rewards']
                        if len(rewards) >= window_size:
                            moving_avg = []
                            for i in range(window_size-1, len(rewards)):
                                moving_avg.append(np.mean(rewards[i-window_size+1:i+1]))
                            
                            ax2.plot(range(window_size-1, len(rewards)), moving_avg, 'b-', label='Reward (moving avg)')
                            ax2.set_xlabel('Training Step')
                            ax2.set_ylabel('Average Reward')
                            ax2.legend()
                        
                        # Show exploration rate for first agent
                        agent = self.rl_controllers[0].agent
                        ax2.text(0.02, 0.98, f"Epsilon: {agent.epsilon:.3f}", 
                               transform=ax2.transAxes, verticalalignment='top',
                               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
                    else:
                        ax2.text(0.5, 0.5, "Switch to RL mode to see training metrics", 
                               transform=ax2.transAxes, ha='center', va='center',
                               fontsize=12, style='italic')
                    
                    plt.tight_layout()
                    plt.pause(0.01)
                
                frame_count += 1
                time.sleep(max(0, self.dt - 0.01))  # Maintain real-time simulation
                
        except KeyboardInterrupt:
            print(f"\n🛑 Demo stopped by user")
            print(f"⏱️ Total runtime: {time.time() - start_time:.1f}s")
            print(f"🎯 Episodes completed: {self.episode}")
            
            # Auto-save models without blocking input
            if self.use_rl and self.training_mode and self.episode > 0:
                print("💾 Auto-saving trained models...")
                try:
                    self.save_models()
                    print("✅ Models saved successfully!")
                except Exception as e:
                    print(f"❌ Failed to save models: {e}")
            
        finally:
            # Clean shutdown
            try:
                plt.close('all')  # Close all matplotlib windows
                plt.ioff()        # Turn off interactive mode
            except:
                pass  # Ignore any cleanup errors

def main():
    """Main function to run the RL leader-follower demo."""
    print("🤖 Reinforcement Learning Leader-Follower System")
    print("=" * 50)
    
    # Create and run the system
    system = RLLeaderFollowerSystem()
    
    # Try to load pre-trained models
    print("🔍 Looking for pre-trained models...")
    system.load_models()
    
    system.run_demo()

if __name__ == "__main__":
    main()