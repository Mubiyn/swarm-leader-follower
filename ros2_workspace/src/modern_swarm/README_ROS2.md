# ROS2 Integration for Modern Swarm Leader-Follower System

This document explains how to use the ROS2 integration of the unified swarm system with real visualization in RViz and Gazebo.

## 🚀 Quick Start

### Prerequisites
- ROS2 (tested with Humble and Iron)
- Python 3.8+
- Required packages: `rclpy`, `geometry_msgs`, `sensor_msgs`, `visualization_msgs`, `cv_bridge`, `tf2_ros`

### 1. Build the ROS2 Workspace
```bash
cd ros2_workspace
colcon build
source install/setup.bash
```

### 2. Run with RViz Visualization
```bash
ros2 launch modern_swarm unified_swarm_rviz.launch.py
```

### 3. Run with Gazebo Simulation
```bash
ros2 launch modern_swarm unified_swarm_gazebo.launch.py
```

### 4. Test the Integration
```bash
# In another terminal
ros2 run modern_swarm test_ros2_integration.py
```

## 📡 ROS2 Topics

### Published Topics
- `/swarm/leader/pose` (geometry_msgs/PoseStamped) - Leader position and orientation
- `/swarm/followers/poses` (geometry_msgs/PoseStamped) - Follower positions and orientations
- `/swarm/formation_targets` (geometry_msgs/Point) - Target positions for each follower
- `/swarm/obstacles` (visualization_msgs/MarkerArray) - Obstacle positions and properties
- `/swarm/camera/image` (sensor_msgs/Image) - Synthetic camera view
- `/swarm/status` (std_msgs/String) - System status and parameters
- `/swarm/robot_markers` (visualization_msgs/MarkerArray) - Robot visualization markers

### Services
- `/swarm/set_formation` (std_msgs/String) - Change formation type
  - Available formations: "triangle", "line", "circle", "v_shape"
- `/swarm/toggle_vision` (std_msgs/Bool) - Enable/disable vision system
- `/swarm/toggle_obstacles` (std_msgs/Bool) - Enable/disable obstacle avoidance
- `/swarm/add_obstacle` (std_msgs/Float32MultiArray) - Add new obstacle
  - Data format: [x, y, radius, is_dynamic, vx, vy] (last two optional for dynamic obstacles)
- `/swarm/set_controller` (std_msgs/String) - Change controller type
  - Available controllers: "Proportional", "MPC"

## 🎮 Interactive Control

### Using Command Line Tools

#### Change Formation
```bash
ros2 service call /swarm/set_formation std_msgs/String "data: 'line'"
```

#### Toggle Vision System
```bash
ros2 service call /swarm/toggle_vision std_msgs/Bool "data: true"
```

#### Toggle Obstacle Avoidance
```bash
ros2 service call /swarm/toggle_obstacles std_msgs/Bool "data: true"
```

#### Add Static Obstacle
```bash
ros2 service call /swarm/add_obstacle std_msgs/Float32MultiArray "data: [5.0, 5.0, 1.0, 0]"
```

#### Add Dynamic Obstacle
```bash
ros2 service call /swarm/add_obstacle std_msgs/Float32MultiArray "data: [3.0, -3.0, 0.8, 1, 0.5, -0.3]"
```

#### Change Controller
```bash
ros2 service call /swarm/set_controller std_msgs/String "data: 'MPC'"
```

### Using RViz
1. Open RViz with the provided configuration
2. Use the 2D Pose Estimate tool to set initial positions
3. Use the 2D Nav Goal tool to set target positions
4. Monitor robot positions, obstacles, and camera view in real-time

## 🔧 Configuration

### ROS2 Parameters
The system can be configured using ROS2 parameters:

```bash
# Set leader speed
ros2 param set /unified_swarm_system leader_speed 1.5

# Set controller gains
ros2 param set /unified_swarm_system kp_distance 3.0
ros2 param set /unified_swarm_system kp_angle 5.0

# Set collision avoidance parameters
ros2 param set /unified_swarm_system min_robot_distance 1.5
```

### Available Parameters
- `update_rate` (float): System update rate in Hz (default: 20.0)
- `leader_speed` (float): Leader movement speed (default: 1.2)
- `leader_radius` (float): Leader circular trajectory radius (default: 8.0)
- `num_followers` (int): Number of follower robots (default: 3)
- `kp_distance` (float): Proportional controller distance gain (default: 2.5)
- `kp_angle` (float): Proportional controller angle gain (default: 4.0)
- `max_linear_vel` (float): Maximum linear velocity (default: 1.0)
- `max_angular_vel` (float): Maximum angular velocity (default: 2.0)
- `min_robot_distance` (float): Minimum safe distance between robots (default: 1.2)
- `obstacle_detection_range` (float): Obstacle detection range (default: 3.0)
- `obstacle_safety_distance` (float): Safety distance from obstacles (default: 1.5)
- `camera_resolution` (list): Camera resolution [width, height] (default: [640, 480])
- `camera_fov` (list): Camera field of view [x_min, x_max, y_min, y_max] (default: [-10.0, 10.0, -10.0, 10.0])

## 🎯 Features

### 1. Formation Control
- **Triangle Formation**: Classic triangular arrangement
- **Line Formation**: Robots in a straight line
- **Circle Formation**: Robots arranged in a circle
- **V-Shape Formation**: V-shaped arrangement

### 2. Computer Vision
- Synthetic camera view generation
- Leader detection and tracking
- Real-time camera feed in RViz
- Vision-based control demonstration

### 3. Obstacle Avoidance
- Static obstacle avoidance
- Dynamic obstacle tracking and avoidance
- Configurable safety distances
- Real-time obstacle visualization

### 4. Robot Collision Prevention
- Prevents robots from passing through each other
- Configurable minimum distance
- Applies to both leader and followers
- Real-time collision detection

### 5. Multiple Controllers
- **Proportional Controller**: Simple and fast
- **MPC Controller**: Advanced model predictive control
- Runtime controller switching
- Configurable controller parameters

## 🔍 Monitoring and Debugging

### View System Status
```bash
ros2 topic echo /swarm/status
```

### Monitor Robot Positions
```bash
ros2 topic echo /swarm/leader/pose
ros2 topic echo /swarm/followers/poses
```

### View Obstacles
```bash
ros2 topic echo /swarm/obstacles
```

### Check Camera Feed
```bash
ros2 run rqt_image_view rqt_image_view
# Then select /swarm/camera/image topic
```

### List All Topics
```bash
ros2 topic list
```

### List All Services
```bash
ros2 service list
```

## 🐛 Troubleshooting

### Common Issues

1. **Import Error for swarm_core**
   ```bash
   # Make sure the Python path includes the demos directory
   export PYTHONPATH=$PYTHONPATH:/path/to/your/project/demos/python
   ```

2. **RViz Not Showing Robots**
   - Check that the fixed frame is set to "map"
   - Verify that robot markers are enabled in RViz
   - Check that the marker topics are being published

3. **Services Not Responding**
   ```bash
   # Check if the swarm system is running
   ros2 node list
   ros2 service list
   ```

4. **Gazebo Not Starting**
   ```bash
   # Install Gazebo if not already installed
   sudo apt install gazebo libgazebo-dev
   ```

### Debug Mode
Run the system with debug output:
```bash
ros2 launch modern_swarm unified_swarm_rviz.launch.py --ros-args --log-level debug
```

## 🚀 Advanced Usage

### Custom Robot Models
To use custom robot models in Gazebo:

1. Create your robot URDF/XACRO file
2. Place it in the `urdf/` directory
3. Update the launch file to spawn your robots
4. Modify the swarm system to use your robot's topics

### Integration with Real Robots
To integrate with real robots:

1. Replace the simulated robot topics with real robot topics
2. Update the robot control interface
3. Modify the sensor data processing
4. Test with a single robot first

### Custom Controllers
To add custom controllers:

1. Implement your controller in the `swarm_core.py` file
2. Add it to the controllers list in the ROS2 node
3. Update the service callback to handle your controller
4. Test with the provided test script

## 📊 Performance Monitoring

### System Performance
Monitor system performance using:
```bash
# CPU and memory usage
ros2 run system_monitor system_monitor

# Topic frequency
ros2 topic hz /swarm/leader/pose
```

### Formation Accuracy
The system publishes formation errors and control efforts that can be monitored for performance analysis.

## 🤝 Contributing

To contribute to the ROS2 integration:

1. Follow the existing code structure
2. Add proper error handling
3. Include documentation for new features
4. Test with the provided test script
5. Update this README with new features

## 📝 License

This ROS2 integration is part of the Modern Swarm Leader-Follower System and follows the same license as the main project. 