# Architecture Overview

This document outlines the system architecture for the modernized multi-robot leader-follower navigation system.

## System Overview

The system is built on a **decentralized, modular architecture** using ROS2, where each robot operates independently while collaborating through message passing.

```
┌─────────────────────────────────────────────────────────────┐
│                    SIMULATION LAYER                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Robot 1   │  │   Robot 2   │  │   Robot N   │        │
│  │  (Leader)   │  │ (Follower)  │  │ (Follower)  │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                     CONTROL LAYER                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Teleop    │  │ MPC Control │  │ RL Control  │        │
│  │ Controller  │  │  Module     │  │   Module    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                     VISION LAYER                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Camera    │  │    YOLO     │  │  Tracking   │        │
│  │  Interface  │  │  Detection  │  │   Module    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 LAYER                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Topics    │  │  Services   │  │   Actions   │        │
│  │   /cmd_vel  │  │  /spawn     │  │ /navigate   │        │
│  │   /image    │  │  /reset     │  │   /track    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

## Component Architecture

### 1. **Simulation Layer** (`swarm_simulation`)

**Purpose**: Manages the Gazebo simulation environment and robot spawning.

**Key Components**:
- `spawn_robots.py`: Dynamic robot spawning with configurable positions
- `world_manager.py`: Environment setup and obstacle management
- `robot_models/`: URDF/SDF files for robots with RGB-D cameras

**Topics**:
- `/gazebo/spawn_entity` (Service)
- `/robot_X/joint_states` (Subscriber)

### 2. **Vision Layer** (`swarm_vision`)

**Purpose**: Handles all computer vision tasks for robot detection and tracking.

**Key Components**:
- `robot_detector.py`: YOLO-based robot detection
- `visual_tracker.py`: Multi-object tracking using DeepSORT
- `camera_calibration.py`: Camera parameter management
- `data_collector.py`: Dataset creation for training

**Architecture**:
```
Camera → Image Preprocessing → YOLO Detection → Tracking → Pose Estimation
                                    ↓
                              Robot Positions → Formation Controller
```

**Topics**:
- `/robot_X/camera/image_raw` (Subscriber)
- `/robot_X/camera/depth` (Subscriber)
- `/robot_X/detected_robots` (Publisher)
- `/robot_X/leader_pose` (Publisher)

### 3. **Control Layer** (`swarm_control`)

**Purpose**: Implements advanced control algorithms for formation and navigation.

**Key Components**:
- `mpc_controller.py`: Model Predictive Control implementation
- `rl_controller.py`: Reinforcement Learning agent
- `formation_manager.py`: Formation pattern management
- `obstacle_avoidance.py`: Dynamic obstacle avoidance

**Control Architecture**:
```
Leader Pose → Formation     → MPC/RL      → cmd_vel
             Calculator        Controller
                ↑                 ↑
Obstacles   → Safety        → Emergency
             Monitor          Override
```

**Topics**:
- `/robot_X/leader_pose` (Subscriber)
- `/robot_X/scan` (Subscriber)
- `/robot_X/cmd_vel` (Publisher)
- `/robot_X/formation_state` (Publisher)

### 4. **Main Package** (`modern_swarm_leader_follower`)

**Purpose**: Orchestrates the entire system and provides high-level interfaces.

**Key Components**:
- `swarm_manager.py`: Central coordination node
- `experiment_runner.py`: Automated experiment execution
- `performance_monitor.py`: System performance tracking
- `config_manager.py`: Parameter management

## Data Flow

### 1. **Perception Pipeline**
```
RGB-D Camera → Image Processing → Robot Detection → Pose Estimation → Control Input
```

### 2. **Control Pipeline**
```
Target Pose → Path Planning → Control Commands → Robot Motion → State Feedback
```

### 3. **Formation Pipeline**
```
Leader Motion → Formation Update → Individual Targets → Local Control → Formation Maintenance
```

## Key Design Decisions

### **Decentralized vs Centralized**
- **Choice**: Decentralized architecture
- **Rationale**: Better scalability, fault tolerance, and real-world applicability
- **Implementation**: Each robot runs independent vision and control nodes

### **Vision Approach**
- **Choice**: CNN-based detection instead of AR tags
- **Rationale**: More realistic, no infrastructure requirements
- **Implementation**: YOLO + DeepSORT for detection and tracking

### **Control Strategy**
- **Choice**: MPC with RL as alternative
- **Rationale**: MPC provides guaranteed stability, RL enables learning
- **Implementation**: Modular controllers that can be swapped

### **Communication**
- **Choice**: ROS2 topics and services
- **Rationale**: Standard robotics middleware with good performance
- **Implementation**: Namespace-based multi-robot support

## Performance Considerations

### **Real-time Requirements**
- Vision: 10-15 Hz for detection, 30 Hz for tracking
- Control: 20-50 Hz for stable control
- Formation: 5-10 Hz for formation updates

### **Scalability**
- Support for 2-10 robots initially
- Modular design allows extension to larger swarms
- Computational load scales linearly with robot count

### **Robustness**
- Graceful degradation when robots are lost
- Fault detection and recovery mechanisms
- Parameter adaptation for different scenarios

## Development Phases

### **Phase 1: Foundation** 
- Basic ROS2 setup and robot spawning
- Simple camera integration
- Teleoperation interface

### **Phase 2: Vision**
- YOLO training and integration
- Multi-robot detection
- Pose estimation pipeline

### **Phase 3: Control**
- MPC controller implementation
- Formation algorithms
- Safety systems

### **Phase 4: Advanced Features**
- RL controller
- Complex formations
- Dynamic environments

### **Phase 5: Integration**
- System testing
- Performance optimization
- Documentation

## Configuration Management

### **Parameters**
- Robot-specific: camera calibration, control gains
- System-wide: formation patterns, safety thresholds
- Environment: obstacle maps, goal positions

### **Files**
- `configs/robot_params.yaml`: Robot-specific parameters
- `configs/formation_params.yaml`: Formation configurations
- `configs/control_params.yaml`: Controller parameters

## Testing Strategy

### **Unit Tests**
- Individual component testing
- Mock interfaces for isolation
- Automated test suites

### **Integration Tests**
- Multi-component interaction
- ROS2 communication testing
- Performance benchmarking

### **System Tests**
- End-to-end scenarios
- Robustness testing
- Comparison with baseline

## Monitoring and Debugging

### **Logging**
- Structured logging with different levels
- Performance metrics collection
- Event tracing for debugging

### **Visualization**
- RViz2 for real-time monitoring
- Custom dashboards for experiments
- Data plotting and analysis tools 