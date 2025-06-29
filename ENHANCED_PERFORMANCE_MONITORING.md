# Enhanced Performance Monitoring for Swarm System

## 🎯 Overview

The swarm system now includes comprehensive performance monitoring that tracks:
- **Formation errors over time** for each robot
- **Collision statistics** and safety metrics
- **Vision system performance** 
- **MPC controller performance**
- **Real-time system health assessment**

## 📊 What's New

### ✅ **Formation Error Tracking**
- **Individual robot tracking**: Each follower's formation error is tracked separately
- **Time-series data**: Formation errors are stored with timestamps for trend analysis
- **Detailed metadata**: Includes target vs actual positions for debugging
- **Average calculations**: System-wide formation error statistics

### ✅ **Collision Statistics**
- **Real-time collision detection**: Monitors robot-to-robot and robot-to-obstacle collisions
- **Near-collision tracking**: Tracks close encounters before actual collisions
- **Collision rates**: Calculates collisions per second over time windows
- **Safety scoring**: Overall system safety assessment

### ✅ **Enhanced Data Collection**
- **Comprehensive metrics**: Position, formation error, collision, vision, and controller metrics
- **Trend analysis**: Automatic detection of improving/worsening performance
- **Alert system**: Real-time alerts for high formation errors or collision risks
- **Performance history**: Long-term data storage for analysis

## 🚀 How to Use

### 1. **Start the Enhanced System**
```bash
cd ros2_workspace
source install/setup.bash
ros2 launch modern_swarm simple_gazebo_swarm.launch.py
```

### 2. **Enable Vision and MPC**
```bash
# Enable vision system
ros2 service call /swarm/toggle_vision std_srvs/srv/SetBool "{data: true}"
ros2 service call /swarm/toggle_vision_detection std_srvs/srv/SetBool "{data: true}"

# Switch to MPC controller
ros2 service call /swarm/set_controller std_srvs/srv/SetBool "{data: true}"
```

### 3. **Monitor Performance**
```bash
# Watch formation errors and collision statistics
ros2 topic echo /swarm/performance_metrics

# Check system status
ros2 topic echo /swarm/status

# Generate performance report
ros2 service call /swarm/logging/generate_report std_srvs/srv/SetBool "{data: true}"
```

### 4. **Run Automated Test**
```bash
# Run the performance test script
python3 test_enhanced_performance.py
```

## 📈 Performance Metrics

### **Formation Error Metrics**
- `formation_error`: Individual robot formation error (distance from target)
- `average_formation_error`: System-wide average formation error
- `formation_trends`: Performance trends (improving/stable/worsening)

### **Collision Metrics**
- `collision_statistics`: Total collisions and near-collisions
- `collision_rate`: Collisions per second over time windows
- `safety_score`: Overall system safety (0-100)

### **Vision Metrics**
- `vision_performance`: Detection rate, accuracy, processing time
- `vision_confidence`: Detection confidence levels
- `vision_detections`: Number of successful detections

### **Controller Metrics**
- `controller_status`: Current controller (Proportional/MPC)
- `performance`: Controller response time and accuracy
- `control_effort`: Energy consumption and control signals

## 🔍 System Health Assessment

### **Health Score Calculation**
- **Base score**: 100 points
- **Formation errors**: -20 points for high errors (>2.0)
- **Collisions**: -30 points for high collision rate (>5), -15 for moderate (>2)
- **Near-collisions**: -10 points for high rate (>20)
- **Performance issues**: -15 points for poor obstacle avoidance, -10 for low vision accuracy
- **Critical events**: -5 points per recent critical event

### **Health Status Levels**
- **Healthy**: Score > 80
- **Warning**: Score 60-80
- **Critical**: Score < 60

## 🎛️ Interactive Control

### **Parameter Tuning**
```bash
# Tune formation control parameters
ros2 service call /swarm/control/set_parameter std_srvs/srv/SetBool "{data: true}"

# Emergency stop
ros2 service call /swarm/control/emergency_stop std_srvs/srv/SetBool "{data: true}"

# Reset parameters to defaults
ros2 service call /swarm/control/reset_parameters std_srvs/srv/SetBool "{data: true}"
```

### **Auto-tuning**
```bash
# Auto-tune controller parameters
ros2 service call /swarm/control/tune_controller std_srvs/srv/SetBool "{data: true}"
```

## 📊 Data Analysis

### **Real-time Analysis**
- **Trend detection**: Automatic identification of performance trends
- **Alert generation**: Real-time alerts for critical issues
- **Recommendation engine**: AI-powered suggestions for improvement

### **Historical Analysis**
- **Performance reports**: Comprehensive analysis reports
- **Data export**: Export metrics for external analysis
- **Visualization**: Performance plots and charts

## 🛡️ Safety Features

### **Collision Prevention**
- **Real-time monitoring**: Continuous collision risk assessment
- **Proactive avoidance**: Enhanced obstacle and robot avoidance
- **Emergency protocols**: Automatic safety measures for critical situations

### **Performance Alerts**
- **Formation error alerts**: Warnings for high formation errors
- **Collision alerts**: Critical alerts for collision events
- **System health alerts**: Notifications for degrading performance

## 📁 Data Storage

### **Logging Structure**
```
logs/
├── metrics/           # Performance metrics
├── events/           # System events and alerts
├── exports/          # Exported data and reports
└── visualizations/   # Performance plots and charts
```

### **Data Retention**
- **Metrics**: Stored with timestamps for trend analysis
- **Events**: Logged with severity levels and descriptions
- **Retention**: Configurable retention period (default: 7 days)

## 🔧 Configuration

### **Performance Parameters**
```yaml
# Formation control
formation_spacing: 2.0
formation_tolerance: 0.3

# Controller parameters
kp_distance: 2.5
kp_angle: 4.0
max_linear_vel: 1.0
max_angular_vel: 2.0

# Safety parameters
min_robot_distance: 1.2
obstacle_safety_distance: 1.5
emergency_stop_distance: 0.5

# Vision parameters
vision_confidence_threshold: 0.6
vision_update_rate: 30.0
```

## 🎯 Benefits

### **For Researchers**
- **Comprehensive data**: Complete performance metrics for analysis
- **Trend analysis**: Long-term performance tracking
- **Reproducible results**: Detailed logging for experiment replication

### **For Developers**
- **Debugging tools**: Detailed error tracking and metadata
- **Performance optimization**: Real-time feedback for tuning
- **Safety monitoring**: Proactive collision and safety monitoring

### **For Operators**
- **Real-time monitoring**: Live performance dashboard
- **Alert system**: Immediate notifications for issues
- **Automated recommendations**: AI-powered improvement suggestions

## 🚀 Next Steps

1. **Run the enhanced system** and observe the new metrics
2. **Test different scenarios** to see performance variations
3. **Analyze the data** to understand system behavior
4. **Use recommendations** to optimize performance
5. **Export data** for further analysis and research

The enhanced performance monitoring provides unprecedented visibility into swarm system behavior, enabling better understanding, optimization, and safety management. 