# System Architecture

## Overview

The Modern Swarm Leader-Follower System follows a modular architecture designed to support both standalone Python development and ROS2 deployment. The architecture emphasizes separation of concerns, reusability, and scalability.

## Design Philosophy

### Evolution from Standalone to ROS2
The system was designed with a clear progression path:
1. **Algorithm Development**: Core algorithms implemented in standalone Python
2. **Modular Design**: Components separated into focused modules
3. **ROS2 Integration**: Modules adapted for ROS2 communication patterns
4. **Unified System**: All components integrated into a single ROS2 node

### Key Design Principles
- **Modularity**: Each component has a single responsibility
- **Reusability**: Components can be used in both standalone and ROS2 contexts
- **Extensibility**: New features can be added without modifying existing code
- **Testability**: Each component can be tested independently

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Environment                        │
├─────────────────────────────────────────────────────────────┤
│                Unified Swarm Node                          │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐  │
│  │ Controllers │   Vision    │  Obstacles  │ Interactive │  │
│  └─────────────┴─────────────┴─────────────┴─────────────┘  │
│  ┌─────────────┬─────────────┐                              │
│  │Data Logging │   Plotter   │                              │
│  └─────────────┴─────────────┘                              │
└─────────────────────────────────────────────────────────────┘
```

### Component Architecture

#### 1. Unified Swarm Node (`unified_swarm_ros2.py`)
**Purpose**: Main system orchestrator and ROS2 interface

**Responsibilities**:
- Initialize and manage all subsystems
- Handle ROS2 topic publishing and subscription
- Process service requests
- Coordinate robot state management
- Manage formation patterns

**Key Methods**:
- `__init__()`: Initialize ROS2 components and subsystems
- `update_robot_positions()`: Update robot positions and publish to topics
- `handle_service_requests()`: Process interactive control requests
- `run()`: Main execution loop

#### 2. Controllers Module (`controllers.py`)
**Purpose**: Implement control algorithms for robot movement

**Components**:
- **PID Controller**: Proportional-Integral-Derivative control
- **MPC Controller**: Model Predictive Control implementation

**Key Features**:
- Configurable control parameters
- Smooth transitions between controllers
- Error handling and bounds checking

#### 3. Vision Module (`vision.py`)
**Purpose**: Computer vision for leader detection and tracking

**Capabilities**:
- Leader robot detection
- Position estimation
- Real-time tracking
- Vision-based formation control

**Implementation Notes**:
- Uses OpenCV for image processing
- Configurable detection parameters
- Fallback mechanisms for detection failures

#### 4. Obstacles Module (`obstacles.py`)
**Purpose**: Obstacle detection and avoidance

**Features**:
- Static obstacle management
- Dynamic obstacle detection
- Collision avoidance algorithms
- Integration with formation maintenance

#### 5. Interactive Module (`interactive.py`)
**Purpose**: Service-based control interface

**Services Provided**:
- Formation pattern switching
- Controller selection
- Vision system toggling
- Obstacle avoidance control

#### 6. Data Logging Module (`data_logging.py`)
**Purpose**: Performance monitoring and metrics collection

**Metrics Collected**:
- Formation errors over time
- Collision statistics
- System performance metrics
- Vision system performance

#### 7. Plotter Module (`plotter.py`)
**Purpose**: Real-time visualization and analysis

**Capabilities**:
- Real-time performance plotting
- Formation visualization
- Performance report generation
- Data export functionality

## Communication Architecture

### ROS2 Topics

#### Published Topics
- `/swarm/robot_positions` (geometry_msgs/PoseArray)
  - Current positions of all robots in the swarm
  - Updated at system frequency (typically 10Hz)

- `/swarm/formation_status` (std_msgs/String)
  - Current formation pattern and status
  - Published when formation changes

- `/swarm/performance_metrics` (std_msgs/String)
  - Performance metrics in JSON format
  - Published at regular intervals

- `/swarm/visualization_markers` (visualization_msgs/MarkerArray)
  - RViz visualization markers
  - Robot positions, formation lines, obstacles

#### Subscribed Topics
- `/swarm/vision_data` (sensor_msgs/Image)
  - Camera feed for vision processing
  - Used by vision module for leader detection

### ROS2 Services

All services use `std_srvs/SetBool` for compatibility with standard ROS2 tools:

- `/swarm/set_formation`
  - Changes formation pattern
  - Data: true = next formation, false = previous formation

- `/swarm/set_controller`
  - Switches between PID and MPC controllers
  - Data: true = MPC, false = PID

- `/swarm/toggle_vision`
  - Enables/disables vision system
  - Data: true = enable, false = disable

- `/swarm/toggle_obstacle_avoidance`
  - Enables/disables obstacle avoidance
  - Data: true = enable, false = disable

- `/swarm/add_obstacle`
  - Adds dynamic obstacles to the environment
  - Data: true = add obstacle, false = remove obstacle

## Data Flow

### Robot Position Update Flow
```
1. Vision Module → Leader Position Detection
2. Controllers Module → Calculate Follower Positions
3. Obstacles Module → Apply Collision Avoidance
4. Unified Node → Update Robot States
5. Unified Node → Publish to Topics
6. Plotter Module → Update Visualizations
```

### Service Request Flow
```
1. ROS2 Service Call → Interactive Module
2. Interactive Module → Validate Request
3. Interactive Module → Update System State
4. Unified Node → Apply Changes
5. Data Logging → Record State Change
```

## Formation Patterns

### Supported Patterns

#### Line Formation
- Robots arranged in a straight line behind the leader
- Configurable spacing between robots
- Maintains orientation relative to leader

#### Circle Formation
- Robots arranged in a circle around the leader
- Configurable radius and angular spacing
- Maintains circular pattern during movement

#### Triangle Formation
- Three robots arranged in triangular pattern
- Configurable base width and height
- Maintains triangular shape during movement

#### Square Formation
- Four robots arranged in square pattern
- Configurable side length
- Maintains square shape during movement

### Formation Management
- Automatic formation calculation based on leader position
- Smooth transitions between formation patterns
- Collision avoidance within formation constraints
- Formation error tracking and correction

## Performance Considerations

### Real-Time Requirements
- System operates at 10Hz update rate
- Vision processing must complete within 100ms
- Controller calculations must complete within 50ms
- Service responses must complete within 10ms

### Memory Management
- Robot state data stored in efficient data structures
- Performance metrics stored with configurable history length
- Visualization data managed to prevent memory leaks
- Regular cleanup of temporary data

### Scalability
- Architecture supports additional robots (tested up to 5 robots)
- Modular design allows for easy addition of new features
- Performance monitoring helps identify bottlenecks
- Configurable parameters for different swarm sizes

## Error Handling

### Vision System Failures
- Fallback to last known leader position
- Automatic retry mechanisms
- Graceful degradation of formation control

### Controller Failures
- Automatic fallback to PID controller
- Error logging and reporting
- System state preservation

### Communication Failures
- Robust ROS2 topic handling
- Service timeout management
- Automatic reconnection attempts

## Configuration Management

### Parameter Files
- Robot parameters in `config/robot_params.yaml`
- Formation parameters in code (configurable)
- Performance monitoring parameters in data logging module

### Environment Variables
- ROS2 environment setup
- Python path configuration
- Debug mode toggles

## Testing Architecture

### Unit Testing
- Individual module testing
- Mock ROS2 components for testing
- Automated test suites

### Integration Testing
- Full system testing with ROS2
- Performance validation
- End-to-end scenario testing

### Demo Testing
- Standalone Python demos for algorithm validation
- ROS2 demos for feature-specific testing
- Performance benchmarking

## Future Architecture Considerations

### Potential Extensions
- Distributed computing for large swarms
- Machine learning integration
- Advanced path planning algorithms
- Real robot hardware integration

### Scalability Improvements
- Multi-node architecture for large swarms
- Database integration for performance data
- Web-based monitoring interface
- Cloud-based computation offloading 