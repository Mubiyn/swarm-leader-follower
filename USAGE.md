# Usage Guide

## Installation

### Prerequisites

1. **ROS2 Installation**
   - ROS2 Humble or later
   - Tested with conda-based ROS2 installation
   - Ensure ROS2 environment is properly sourced

2. **Python Requirements**
   - Python 3.8 or later
   - Required packages listed in `requirements.txt`

3. **System Requirements**
   - macOS (tested on macOS 24.1.0)
   - Sufficient RAM for ROS2 operations (4GB+ recommended)
   - Display support for RViz visualization

### Setup Steps

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd modern_swarm_leader_follower
   ```

2. **Install Python Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Build ROS2 Workspace**
   ```bash
   cd ros2_workspace
   colcon build
   source install/setup.bash
   cd ..
   ```

4. **Verify Installation**
   ```bash
   # Check if ROS2 nodes are available
   ros2 node list
   
   # Check if services are available
   ros2 service list
   ```

## Running the System

### Quick Start

1. **Start the Main Demo**
   ```bash
   ./run_swarm_ros2_demo.sh
   ```

2. **Launch RViz Visualization**
   ```bash
   ros2 launch modern_swarm unified_swarm_rviz.launch.py
   ```

3. **Enable Features**
   ```bash
   # Enable vision system
   ros2 service call /swarm/toggle_vision std_srvs/srv/SetBool "data: true"
   
   # Switch to MPC controller
   ros2 service call /swarm/set_controller std_srvs/srv/SetBool "data: true"
   ```

### Manual Startup

1. **Source ROS2 Environment**
   ```bash
   source ros2_workspace/install/setup.bash
   ```

2. **Set Python Path**
   ```bash
   export PYTHONPATH=$PYTHONPATH:$(pwd)/demos/python
   ```

3. **Run the Unified System**
   ```bash
   ros2 run modern_swarm unified_swarm_ros2.py
   ```

4. **Launch RViz**
   ```bash
   ros2 launch modern_swarm unified_swarm_rviz.launch.py
   ```

## System Control

### Available Services

The system provides several ROS2 services for interactive control:

#### Formation Control
```bash
# Change formation pattern (cycles through: line → circle → triangle → square)
ros2 service call /swarm/set_formation std_srvs/srv/SetBool "data: true"

# Go to previous formation pattern
ros2 service call /swarm/set_formation std_srvs/srv/SetBool "data: false"
```

#### Controller Selection
```bash
# Switch to MPC controller
ros2 service call /swarm/set_controller std_srvs/srv/SetBool "data: true"

# Switch to PID controller
ros2 service call /swarm/set_controller std_srvs/srv/SetBool "data: false"
```

#### Vision System
```bash
# Enable vision system
ros2 service call /swarm/toggle_vision std_srvs/srv/SetBool "data: true"

# Disable vision system
ros2 service call /swarm/toggle_vision std_srvs/srv/SetBool "data: false"
```

#### Obstacle Avoidance
```bash
# Enable obstacle avoidance
ros2 service call /swarm/toggle_obstacle_avoidance std_srvs/srv/SetBool "data: true"

# Disable obstacle avoidance
ros2 service call /swarm/toggle_obstacle_avoidance std_srvs/srv/SetBool "data: false"

# Add dynamic obstacle
ros2 service call /swarm/add_obstacle std_srvs/srv/SetBool "data: true"

# Remove dynamic obstacle
ros2 service call /swarm/add_obstacle std_srvs/srv/SetBool "data: false"
```

### Monitoring Topics

#### Robot Positions
```bash
# Monitor robot positions
ros2 topic echo /swarm/robot_positions
```

#### Formation Status
```bash
# Monitor formation changes
ros2 topic echo /swarm/formation_status
```

#### Performance Metrics
```bash
# Monitor performance data
ros2 topic echo /swarm/performance_metrics
```

#### Visualization Markers
```bash
# Monitor visualization data
ros2 topic echo /swarm/visualization_markers
```

## Demo Scripts

### Standalone Python Demos

The `demos/python/` folder contains standalone implementations for testing individual features:

#### MPC Controller Demo
```bash
cd demos/python
python mpc_leader_follower.py
```

#### Vision Demo
```bash
cd demos/python
python vision_leader_follower.py
```

#### Obstacle Avoidance Demo
```bash
cd demos/python
python obstacle_avoidance_demo.py
```

### ROS2 Feature Demos

The `demos/ros2/` folder contains ROS2-specific demonstrations:

#### Comprehensive Bridge Demo
```bash
cd demos/ros2
python ros2_swarm_bridge_with_services.py
```

#### Vision System Demo
```bash
cd demos/ros2
python vision_leader_follower_ros2.py
```

#### Multi-Follower Demo
```bash
cd demos/ros2
python multi_follower_ros2.py
```

## Configuration

### Robot Parameters

Edit `ros2_workspace/src/modern_swarm/config/robot_params.yaml` to modify:

- Robot dimensions and properties
- Formation spacing parameters
- Control algorithm parameters
- Performance monitoring settings

### Formation Parameters

Formation parameters are configurable in the code:

- **Line Formation**: Spacing between robots
- **Circle Formation**: Radius and angular spacing
- **Triangle Formation**: Base width and height
- **Square Formation**: Side length

### Performance Monitoring

Performance monitoring parameters can be adjusted in `data_logging.py`:

- Metrics collection frequency
- History length for performance data
- Visualization update rate
- Report generation settings

## Visualization

### RViz Configuration

The system includes several RViz configuration files:

- `simple_swarm.rviz`: Basic visualization
- `swarm_visualization.rviz`: Enhanced visualization with markers
- `working_swarm.rviz`: Production-ready configuration

### Performance Plots

Performance data is automatically generated and stored in `ros2_workspace/performance_plots/`:

- Real-time performance metrics
- Formation error analysis
- Collision statistics
- System health reports

## Troubleshooting

### Common Issues

#### Build Errors
```bash
# Clean build directory
cd ros2_workspace
rm -rf build install log
colcon build
```

#### Import Errors
```bash
# Ensure Python path is set correctly
export PYTHONPATH=$PYTHONPATH:$(pwd)/demos/python
```

#### Service Call Failures
```bash
# Check if services are available
ros2 service list | grep swarm

# Check service type
ros2 service type /swarm/set_formation
```

#### RViz Not Showing Robots
```bash
# Check if markers are being published
ros2 topic echo /swarm/visualization_markers

# Verify RViz configuration
ros2 launch modern_swarm unified_swarm_rviz.launch.py
```

### Debug Mode

Enable debug mode for detailed logging:

```bash
# Set debug environment variable
export SWARM_DEBUG=1

# Run with verbose output
ros2 run modern_swarm unified_swarm_ros2.py --ros-args --log-level debug
```

### Performance Monitoring

Monitor system performance:

```bash
# Check CPU and memory usage
top -p $(pgrep -f unified_swarm_ros2)

# Monitor ROS2 performance
ros2 run performance_test_fixture performance_test
```

## Advanced Usage

### Custom Formations

To add custom formation patterns:

1. Modify the formation calculation in `unified_swarm_ros2.py`
2. Add formation parameters to configuration
3. Update formation switching logic

### Custom Controllers

To implement custom controllers:

1. Add controller class to `controllers.py`
2. Implement required interface methods
3. Update controller selection logic

### Integration with Real Robots

To integrate with real robot hardware:

1. Replace simulation data with real sensor data
2. Implement hardware-specific drivers
3. Add safety checks and emergency stops
4. Test thoroughly in controlled environment

### Performance Optimization

For large swarms or high-frequency operation:

1. Optimize algorithm implementations
2. Use efficient data structures
3. Implement parallel processing where possible
4. Monitor and tune system parameters

## Best Practices

### Development Workflow

1. **Test in Standalone Mode**: Use Python demos for algorithm development
2. **Incremental Integration**: Add features one at a time to ROS2 system
3. **Performance Monitoring**: Always monitor system performance
4. **Error Handling**: Implement robust error handling and recovery

### System Maintenance

1. **Regular Updates**: Keep ROS2 and dependencies updated
2. **Performance Monitoring**: Monitor system performance regularly
3. **Log Analysis**: Analyze logs for potential issues
4. **Backup Configuration**: Keep backup copies of working configurations

### Safety Considerations

1. **Simulation First**: Always test in simulation before real deployment
2. **Emergency Stops**: Implement emergency stop mechanisms
3. **Collision Avoidance**: Ensure collision avoidance is always enabled
4. **Performance Limits**: Respect system performance limits 